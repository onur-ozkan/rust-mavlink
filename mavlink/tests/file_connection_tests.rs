mod test_shared;

#[cfg(feature = "common")]
mod test_file_connections {
    use mavlink::ardupilotmega::MavMessage;

    #[test]
    pub fn test_file_read_raw() {
        let tlog = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/log.tlog")
            .canonicalize()
            .unwrap();

        let tlog = tlog.to_str().unwrap();

        let filename = std::path::Path::new(tlog);
        let filename = filename.to_str().unwrap();
        dbg!(filename);

        println!("Processing file: {filename}");
        let connection_string = format!("file:{filename}");

        println!("connection_string - {connection_string}");

        let vehicle =
            mavlink::connect::<MavMessage>(&connection_string).expect("Couldn't read from file");

        let mut counter = 0;
        loop {
            match vehicle.recv_raw() {
                Ok(raw_msg) => {
                    println!(
                        "raw_msg.component_id() {} | sequence number {} | message_id {:?}",
                        raw_msg.component_id(),
                        raw_msg.sequence(),
                        raw_msg.message_id()
                    );

                    counter += 1;
                }
                Err(mavlink::error::MessageReadError::Io(e)) => {
                    if e.kind() == std::io::ErrorKind::UnexpectedEof {
                        break;
                    }
                }
                _ => {
                    break;
                }
            }
        }

        println!("Number of parsed messages: {counter}");
        assert!(
            counter == 1426,
            "Unable to hit the necessary amount of matches"
        );
    }
}
