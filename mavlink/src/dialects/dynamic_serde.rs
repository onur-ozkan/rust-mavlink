//! Serialization support for runtime-loaded MAVLink messages.

use mavlink_bindgen::parser::MavType;
use serde::ser::{Error as _, Serialize, SerializeMap, SerializeTuple};

use super::dynamic::{DynamicEnumDefinition, DynamicMessage};

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
                    enumeration: field.enumeration(),
                    bytes: available_bytes(self.payload(), field.offset(), field.encoded_size()),
                },
            )?;
        }
        message.end()
    }
}

struct SerializableField<'a> {
    mavtype: &'a MavType,
    enumeration: Option<&'a DynamicEnumDefinition>,
    bytes: &'a [u8],
}

impl Serialize for SerializableField<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        if let MavType::Array(element_type, length) = self.mavtype {
            let element_size = element_type.size();
            let mut sequence = serializer.serialize_tuple(*length)?;
            for index in 0..*length {
                let offset = index * element_size;
                sequence.serialize_element(&Self {
                    mavtype: element_type,
                    enumeration: None,
                    bytes: available_bytes(self.bytes, offset, element_size),
                })?;
            }
            return sequence.end();
        }

        if let MavType::CharArray(length) = self.mavtype {
            let length = (*length).min(self.bytes.len());
            let bytes = &self.bytes[..length];
            let bytes = &bytes[..bytes.iter().position(|byte| *byte == 0).unwrap_or(length)];
            return serializer
                .serialize_str(std::str::from_utf8(bytes).map_err(serde::ser::Error::custom)?);
        }

        if let Some(enumeration) = self.enumeration {
            let value = integer_value(self.mavtype, self.bytes).ok_or_else(|| {
                S::Error::custom(format!(
                    "enum {} must use an integer field type",
                    enumeration.name
                ))
            })?;

            if enumeration.bitmask {
                if serializer.is_human_readable() {
                    return serializer.serialize_str(&bitmask_names(enumeration, value));
                }
            } else {
                let entry = enumeration
                    .entries
                    .iter()
                    .find(|(_, entry_value)| *entry_value == value)
                    .ok_or_else(|| {
                        S::Error::custom(format!(
                            "invalid enum value for {}: {value}",
                            enumeration.name
                        ))
                    })?;
                let mut value = serializer.serialize_map(Some(1))?;
                value.serialize_entry("type", &entry.0)?;
                return value.end();
            }
        }

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
            MavType::CharArray(_) | MavType::Array(_, _) => unreachable!(),
        }
    }
}

fn integer_value(mavtype: &MavType, bytes: &[u8]) -> Option<u64> {
    match mavtype {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Int8 | MavType::Char => {
            Some(bytes.first().copied().unwrap_or_default().into())
        }
        MavType::UInt16 | MavType::Int16 => Some(u16::from_le_bytes(padded(bytes)).into()),
        MavType::UInt32 | MavType::Int32 => Some(u32::from_le_bytes(padded(bytes)).into()),
        MavType::UInt64 | MavType::Int64 => Some(u64::from_le_bytes(padded(bytes))),
        MavType::Float | MavType::Double | MavType::CharArray(_) | MavType::Array(_, _) => None,
    }
}

fn bitmask_names(enumeration: &DynamicEnumDefinition, value: u64) -> String {
    let mut names = Vec::new();
    let mut remaining = value;

    for (name, bits) in &enumeration.entries {
        let bits = *bits;
        if bits != 0 && value & bits == bits && remaining & bits != 0 {
            names.push(name.clone());
            remaining &= !bits;
        }
    }

    if remaining != 0 {
        names.push(format!("0x{remaining:x}"));
    }

    names.join(" | ")
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
