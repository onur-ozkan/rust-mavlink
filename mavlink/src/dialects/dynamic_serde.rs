//! Serialization support for runtime-loaded MAVLink messages.

use mavlink_bindgen::parser::MavType;
use serde::ser::{Serialize, SerializeMap, SerializeSeq};

use super::dynamic::DynamicMessage;

impl Serialize for DynamicMessage {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let definition = self.definition();
        let mut message = serializer.serialize_map(Some(definition.fields().len() + 1))?;
        message.serialize_entry("type", definition.name())?;
        for field in definition.fields() {
            message.serialize_entry(
                field.name(),
                &SerializableField {
                    mavtype: field.mavtype(),
                    bytes: available_bytes(self.payload(), field.offset(), field.encoded_size()),
                },
            )?;
        }
        message.end()
    }
}

struct SerializableField<'a> {
    mavtype: &'a MavType,
    bytes: &'a [u8],
}

impl Serialize for SerializableField<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match self.mavtype {
            MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => {
                serializer.serialize_u8(self.bytes.first().copied().unwrap_or_default())
            }
            MavType::UInt16 => serializer.serialize_u16(u16::from_le_bytes(padded(self.bytes))),
            MavType::UInt32 => serializer.serialize_u32(u32::from_le_bytes(padded(self.bytes))),
            MavType::UInt64 => serializer.serialize_u64(u64::from_le_bytes(padded(self.bytes))),
            MavType::Int8 => {
                serializer.serialize_i8(self.bytes.first().copied().unwrap_or_default() as i8)
            }
            MavType::Int16 => serializer.serialize_i16(i16::from_le_bytes(padded(self.bytes))),
            MavType::Int32 => serializer.serialize_i32(i32::from_le_bytes(padded(self.bytes))),
            MavType::Int64 => serializer.serialize_i64(i64::from_le_bytes(padded(self.bytes))),
            MavType::Float => serializer.serialize_f32(f32::from_le_bytes(padded(self.bytes))),
            MavType::Double => serializer.serialize_f64(f64::from_le_bytes(padded(self.bytes))),
            MavType::CharArray(length) => {
                let length = (*length).min(self.bytes.len());
                let bytes = &self.bytes[..length];
                let bytes = &bytes[..bytes.iter().position(|byte| *byte == 0).unwrap_or(length)];
                serializer
                    .serialize_str(std::str::from_utf8(bytes).map_err(serde::ser::Error::custom)?)
            }
            MavType::Array(element_type, length) => {
                let element_size = element_type.size();
                let mut sequence = serializer.serialize_seq(Some(*length))?;
                for index in 0..*length {
                    let offset = index * element_size;
                    sequence.serialize_element(&Self {
                        mavtype: element_type,
                        bytes: available_bytes(self.bytes, offset, element_size),
                    })?;
                }
                sequence.end()
            }
        }
    }
}

fn available_bytes(bytes: &[u8], offset: usize, length: usize) -> &[u8] {
    let bytes = bytes.get(offset..).unwrap_or_default();
    &bytes[..bytes.len().min(length)]
}

fn padded<const N: usize>(bytes: &[u8]) -> [u8; N] {
    let mut padded = [0; N];
    let length = N.min(bytes.len());
    padded[..length].copy_from_slice(&bytes[..length]);
    padded
}
