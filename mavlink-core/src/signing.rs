use crate::MAVLinkV2MessageRaw;

use std::time::SystemTime;
use std::{collections::HashMap, sync::Mutex};

use crate::MAVLINK_IFLAG_SIGNED;

/// Configuration used for MAVLink 2 messages signing as defined in <https://mavlink.io/en/guide/message_signing.html>.
///
/// To use a [`SigningConfig`] for sending and reciving messages create a [`SigningData`] object using `SigningData::from_config`.
///
/// # Examples
/// Creating `SigningData`:
/// ```
/// # use mavlink_core::{SigningData, SigningConfig};
/// let config = SigningConfig::new([0u8; 32], 0, true, false);
/// let sign_data = SigningData::from_config(config);
/// ```
///
#[derive(Debug, Clone)]
pub struct SigningConfig {
    secret_key: [u8; 32],
    link_id: u8,
    pub(crate) sign_outgoing: bool,
    pub(crate) allow_unsigned: bool,
}

// mutable state of signing per connection
pub(crate) struct SigningState {
    timestamp: u64,
    stream_timestamps: HashMap<(u8, u8, u8), u64>,
}

/// MAVLink 2 message signing data
///
/// Contains a [`SigningConfig`] as well as a mutable state that is reused for all messages in a connection.  
pub struct SigningData {
    pub(crate) config: SigningConfig,
    pub(crate) state: Mutex<SigningState>,
}

impl SigningConfig {
    /// Creates a new signing configuration.
    ///
    /// If `sign_outgoing` is set messages send using this configuration will be signed.
    /// If `allow_unsigned` is set, when receiving messages, all unsigned messages are accepted, this may also includes MAVLink 1 messages.
    pub fn new(
        secret_key: [u8; 32],
        link_id: u8,
        sign_outgoing: bool,
        allow_unsigned: bool,
    ) -> Self {
        Self {
            secret_key,
            link_id,
            sign_outgoing,
            allow_unsigned,
        }
    }
}

impl SigningData {
    /// Initializes signing data from a given [`SigningConfig`]
    pub fn from_config(config: SigningConfig) -> Self {
        Self {
            config,
            state: Mutex::new(SigningState {
                timestamp: 0,
                stream_timestamps: HashMap::new(),
            }),
        }
    }

    /// Verify the signature of a MAVLink 2 message.
    ///
    /// This respects the `allow_unsigned` parameter in [`SigningConfig`].
    pub fn verify_signature(&self, message: &MAVLinkV2MessageRaw) -> bool {
        // The code that holds the mutex lock is not expected to panic, therefore the expect is justified.
        // The only issue that might cause a panic, presuming the opertions on the message buffer are sound,
        // is the `SystemTime::now()` call in `get_current_timestamp()`.
        let mut state = self
            .state
            .lock()
            .expect("Code holding MutexGuard should not panic.");
        if message.incompatibility_flags() & MAVLINK_IFLAG_SIGNED > 0 {
            state.timestamp = u64::max(state.timestamp, Self::get_current_timestamp());
            let timestamp = message.signature_timestamp();
            let src_system = message.system_id();
            let src_component = message.component_id();
            let stream_key = (message.signature_link_id(), src_system, src_component);
            match state.stream_timestamps.get(&stream_key) {
                Some(stream_timestamp) => {
                    if timestamp <= *stream_timestamp {
                        // reject old timestamp
                        return false;
                    }
                }
                None => {
                    if timestamp + 60 * 1000 * 100 < state.timestamp {
                        // bad new stream, more then a minute older the the last one
                        return false;
                    }
                }
            }

            let mut signature_buffer = [0u8; 6];
            message.calculate_signature(&self.config.secret_key, &mut signature_buffer);
            let result = signature_buffer == message.signature_value();
            if result {
                // if signature is valid update timestamps
                state.stream_timestamps.insert(stream_key, timestamp);
                state.timestamp = u64::max(state.timestamp, timestamp)
            }
            result
        } else {
            self.config.allow_unsigned
        }
    }

    /// Sign a MAVLink 2 message if its incompatibility flag is set accordingly.
    pub fn sign_message(&self, message: &mut MAVLinkV2MessageRaw) {
        if message.incompatibility_flags() & MAVLINK_IFLAG_SIGNED > 0 {
            // The code that holds the mutex lock is not expected to panic, therefore the expect is justified.
            // The only issue that might cause a panic, presuming the opertions on the message buffer are sound,
            // is the `SystemTime::now()` call in `get_current_timestamp()`.
            let mut state = self
                .state
                .lock()
                .expect("Code holding MutexGuard should not panic.");
            state.timestamp = u64::max(state.timestamp, Self::get_current_timestamp());
            let ts_bytes = u64::to_le_bytes(state.timestamp);
            message
                .signature_timestamp_bytes_mut()
                .copy_from_slice(&ts_bytes[0..6]);
            *message.signature_link_id_mut() = self.config.link_id;

            let mut signature_buffer = [0u8; 6];
            message.calculate_signature(&self.config.secret_key, &mut signature_buffer);

            message
                .signature_value_mut()
                .copy_from_slice(&signature_buffer);
            state.timestamp += 1;
        }
    }

    fn get_current_timestamp() -> u64 {
        // fallback to 0 if the system time appears to be before epoch
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map(|n| n.as_micros())
            .unwrap_or(0);
        // use 1st January 2015 GMT as offset, fallback to 0 if before that date, the used 48 bit of this will overflow in 2104
        ((now
            .checked_sub(1420070400u128 * 1000000u128)
            .unwrap_or_default())
            / 10u128) as u64
    }
}
