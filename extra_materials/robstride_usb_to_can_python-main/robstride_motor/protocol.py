from .types import Type0Message, Type2Message, TypeMessage, Type17Message


class RobstrideProtocolWrapper:
    """
    Protocol definitions for Robstride motor communication.
    """

    HEADER = b'\x41\x54'  # 'AT' in ASCII
    FOOTER = b'\x0d\x0a'  # Carriage return and line feed

    def __init__(self):
        pass

    def decode_all_messages(self, raw_data: bytes) -> list[TypeMessage]:
        """
        Decode multiple messages from raw data.
        
        Args:
            raw_data (bytes): Raw bytes that may contain multiple messages.
        Returns:
            list[TypeMessage]: List of decoded messages.
        """
        messages = []
        offset = 0
        
        while offset < len(raw_data):
            # Find the next header
            header_pos = raw_data.find(self.HEADER, offset)
            if header_pos == -1:
                break
            
            # Find the footer after the header
            footer_pos = raw_data.find(self.FOOTER, header_pos)
            if footer_pos == -1:
                break
            
            # Extract the complete message (including header and footer)
            message = raw_data[header_pos:footer_pos + len(self.FOOTER)]
            
            try:
                decoded = self._decode_single_message(message)
                messages.append(decoded)
            except ValueError:
                # Skip invalid messages and continue
                pass
            
            offset = footer_pos + len(self.FOOTER)
        
        return messages

    def _decode_single_message(self, message: bytes) -> TypeMessage:
        """
        Decode a single message received from the Robstride motor.

        Args:
            message (bytes): The raw message bytes.
        Returns:
            TypeMessage: Decoded message object.
        """
        if (not message.startswith(self.HEADER) or
                not message.endswith(self.FOOTER)):
            raise ValueError("Invalid message format")

        content = message[len(self.HEADER):-len(self.FOOTER)]
        # The first 32 bits represents comm type
        shifted_header = int.from_bytes(content[:4], 'big') >> 3
        shifted_bytes = shifted_header.to_bytes(4, 'big')

        type_ = shifted_bytes[0]

        match type_:
            case 0:
                return Type0Message.from_decoded_fields(
                    header=shifted_bytes,
                    payload=content[5:]
                )
            case 2:
                return Type2Message.from_decoded_fields(
                    header=shifted_bytes,
                    payload=content[5:]
                )
            case 17:
                return Type17Message.from_decoded_fields(
                    header=shifted_bytes,
                    payload=content[5:]
                )
            case _:
                raise NotImplementedError(f"Message type {type_}"
                                          f" not implemented")

    def decode_message(self, message: bytes) -> TypeMessage:
        """
        Decode a single message received from the Robstride motor.
        For backward compatibility. Use decode_all_messages() to handle multiple messages.

        Args:
            message (bytes): The raw message bytes.
        Returns:
            TypeMessage: Decoded message object.
        """
        return self._decode_single_message(message)

