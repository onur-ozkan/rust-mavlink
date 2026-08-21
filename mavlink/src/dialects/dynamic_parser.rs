//! Field value conversion for runtime loaded dialects.

use mavlink_bindgen::parser::{MavEnum, MavType};
use mavlink_core::bytes_mut::BytesMut;

/// A field value used to construct a runtime loaded MAVLink message.
///
/// Strings represent character arrays and symbolic enum or bitmask values.
/// Bitmask names use the same `FLAG_A | FLAG_B` form as generated dialects.
#[derive(Debug, Clone, PartialEq)]
pub enum DynamicValue {
    /// Unsigned integer value.
    Unsigned(u64),
    /// Signed integer value.
    Signed(i64),
    /// Floating-point value.
    Float(f64),
    /// Character array, enum name or `|` separated bitmask names.
    String(String),
    /// Array value where the length is validated against its runtime dialect.
    Array(Vec<Self>),
}

macro_rules! impl_dynamic_value_from {
    ($variant:ident: $($type:ty),+ $(,)?) => {
        $(
            impl From<$type> for DynamicValue {
                fn from(value: $type) -> Self {
                    Self::$variant(value.into())
                }
            }
        )+
    };
}

impl_dynamic_value_from!(Unsigned: u8, u16, u32, u64);
impl_dynamic_value_from!(Signed: i8, i16, i32, i64);
impl_dynamic_value_from!(Float: f32, f64);

impl From<String> for DynamicValue {
    fn from(value: String) -> Self {
        Self::String(value)
    }
}

impl From<&str> for DynamicValue {
    fn from(value: &str) -> Self {
        Self::String(value.to_owned())
    }
}

impl<T: Into<Self>> From<Vec<T>> for DynamicValue {
    fn from(value: Vec<T>) -> Self {
        Self::Array(value.into_iter().map(Into::into).collect())
    }
}

impl<T: Into<Self>, const N: usize> From<[T; N]> for DynamicValue {
    fn from(value: [T; N]) -> Self {
        Self::Array(value.into_iter().map(Into::into).collect())
    }
}

pub(super) fn write_value(
    mavtype: &MavType,
    enumeration: Option<&MavEnum>,
    value: &DynamicValue,
    output: &mut [u8],
) -> Result<(), String> {
    if let MavType::Array(element_type, length) = mavtype {
        let DynamicValue::Array(values) = value else {
            return Err(format!("expected an array of {length} values"));
        };
        if values.len() != *length {
            return Err(format!(
                "expected an array of {length} values, got {}",
                values.len()
            ));
        }

        let element_size = element_type.size();
        for (element, output) in values.iter().zip(output.chunks_exact_mut(element_size)) {
            write_value(element_type, enumeration, element, output)?;
        }
        return Ok(());
    }

    if let MavType::CharArray(length) = mavtype {
        return write_char_array(*length, value, output);
    }

    let mut output = BytesMut::new(output);
    match mavtype {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => output.put_u8(
            u8::try_from(unsigned_value(value, enumeration)?)
                .map_err(|_| "value does not fit in uint8_t".to_owned())?,
        ),
        MavType::UInt16 => output.put_u16_le(
            u16::try_from(unsigned_value(value, enumeration)?)
                .map_err(|_| "value does not fit in uint16_t".to_owned())?,
        ),
        MavType::UInt32 => output.put_u32_le(
            u32::try_from(unsigned_value(value, enumeration)?)
                .map_err(|_| "value does not fit in uint32_t".to_owned())?,
        ),
        MavType::UInt64 => output.put_u64_le(unsigned_value(value, enumeration)?),
        MavType::Int8 => output.put_i8(
            i8::try_from(signed_value(value, enumeration)?)
                .map_err(|_| "value does not fit in int8_t".to_owned())?,
        ),
        MavType::Int16 => output.put_i16_le(
            i16::try_from(signed_value(value, enumeration)?)
                .map_err(|_| "value does not fit in int16_t".to_owned())?,
        ),
        MavType::Int32 => output.put_i32_le(
            i32::try_from(signed_value(value, enumeration)?)
                .map_err(|_| "value does not fit in int32_t".to_owned())?,
        ),
        MavType::Int64 => output.put_i64_le(signed_value(value, enumeration)?),
        MavType::Float => output.put_f32_le(float_value(value)? as f32),
        MavType::Double => output.put_f64_le(float_value(value)?),
        MavType::CharArray(_) | MavType::Array(_, _) => unreachable!(),
    }
    Ok(())
}

pub(super) fn write_default(
    mavtype: &MavType,
    enumeration: Option<&MavEnum>,
    dialect_version: Option<u8>,
    output: &mut [u8],
) -> Result<(), String> {
    if let MavType::Array(element_type, _) = mavtype {
        let element_size = element_type.size();
        for output in output.chunks_exact_mut(element_size) {
            write_default(element_type, None, dialect_version, output)?;
        }
    } else if let Some(value) = enumeration
        .and_then(|enumeration| enumeration.entries_with_values().next())
        .map(|(_, value)| value)
    {
        write_value(mavtype, enumeration, &DynamicValue::Unsigned(value), output)?;
    } else if matches!(mavtype, MavType::UInt8MavlinkVersion) {
        write_value(
            mavtype,
            None,
            &DynamicValue::Unsigned(dialect_version.unwrap_or(0).into()),
            output,
        )?;
    }
    Ok(())
}

fn write_char_array(length: usize, value: &DynamicValue, output: &mut [u8]) -> Result<(), String> {
    match value {
        DynamicValue::String(value) => {
            let value = value.as_bytes();
            let copied = value.len().min(length);
            output.fill(0);
            output[..copied].copy_from_slice(&value[..copied]);
            Ok(())
        }
        DynamicValue::Array(values) if values.len() == length => {
            for (value, byte) in values.iter().zip(output) {
                *byte = u8::try_from(unsigned_value(value, None)?).map_err(|_| {
                    format!("character array element does not fit in uint8_t: {value:?}")
                })?;
            }
            Ok(())
        }
        DynamicValue::Array(values) => Err(format!(
            "expected a character array of {length} values, got {}",
            values.len()
        )),
        _ => Err(format!("expected a string or array of {length} bytes")),
    }
}

fn unsigned_value(value: &DynamicValue, enumeration: Option<&MavEnum>) -> Result<u64, String> {
    let value = match value {
        DynamicValue::Unsigned(value) => *value,
        DynamicValue::Signed(value) => {
            u64::try_from(*value).map_err(|_| "expected a non-negative integer value".to_owned())?
        }
        DynamicValue::String(value) => symbolic_value(value, enumeration)?,
        _ => return Err("expected an integer or symbolic value".to_owned()),
    };
    validate_enum_value(value, enumeration)?;
    Ok(value)
}

fn signed_value(value: &DynamicValue, enumeration: Option<&MavEnum>) -> Result<i64, String> {
    let value = match value {
        DynamicValue::Signed(value) => *value,
        DynamicValue::Unsigned(value) => i64::try_from(*value)
            .map_err(|_| "value does not fit in a signed integer".to_owned())?,
        DynamicValue::String(value) => i64::try_from(symbolic_value(value, enumeration)?)
            .map_err(|_| "symbolic value does not fit in a signed integer".to_owned())?,
        _ => return Err("expected an integer or symbolic value".to_owned()),
    };

    if value >= 0 {
        validate_enum_value(value as u64, enumeration)?;
    } else if enumeration.is_some() {
        return Err("enum values cannot be negative".to_owned());
    }
    Ok(value)
}

fn float_value(value: &DynamicValue) -> Result<f64, String> {
    match value {
        DynamicValue::Float(value) => Ok(*value),
        DynamicValue::Unsigned(value) => Ok(*value as f64),
        DynamicValue::Signed(value) => Ok(*value as f64),
        _ => Err("expected a numeric value".to_owned()),
    }
}

fn symbolic_value(value: &str, enumeration: Option<&MavEnum>) -> Result<u64, String> {
    let enumeration = enumeration.ok_or_else(|| "field has no associated enum".to_owned())?;
    if enumeration.bitmask {
        value.split('|').try_fold(0, |bits, name| {
            enum_entry_value(enumeration, name.trim()).map(|value| bits | value)
        })
    } else {
        enum_entry_value(enumeration, value)
    }
}

fn enum_entry_value(enumeration: &MavEnum, name: &str) -> Result<u64, String> {
    enumeration
        .entries_with_values()
        .find_map(|(entry, value)| (entry.name == name).then_some(value))
        .ok_or_else(|| format!("unknown enum entry {name}"))
}

fn validate_enum_value(value: u64, enumeration: Option<&MavEnum>) -> Result<(), String> {
    let Some(enumeration) = enumeration else {
        return Ok(());
    };
    if enumeration.bitmask
        || enumeration
            .entries_with_values()
            .any(|(_, entry_value)| entry_value == value)
    {
        Ok(())
    } else {
        Err(format!("{value} is not a valid enum value"))
    }
}
