#include "data_stream.h"
#include <string.h>

static const data_stream_header_t data_stream_header_template = {{'$'}};

static data_stream_return_t data_stream_message_validate(const data_stream_message_struct*);
static data_stream_return_t data_stream_message_init(data_stream_message_struct*, data_stream_operation_t, data_stream_id_t, uint8_t);
static data_stream_return_t data_stream_compare_stream_metadata(const data_stream_metadata_t*, const data_stream_metadata_t*);
static uint8_t CRC_xor(const uint8_t*, size_t);

static data_stream_return_t data_stream_file_stream_generic_write(data_stream_file_stream_generic* file_handle, const uint8_t* buffer, uint32_t btw, uint32_t* bw, uint32_t timeout) {
	size_t bytes_written = file_handle->file_write_callback(file_handle->handle, (const void*) buffer, (size_t) btw, timeout);
	*bw = (uint32_t) bytes_written;
	if (bytes_written == 0)
		return data_stream_file_error;
	return data_stream_ok;
}
static data_stream_return_t data_stream_file_stream_generic_read(data_stream_file_stream_generic* file_handle, uint8_t* buffer, uint32_t btr, uint32_t*br, uint32_t timeout) {
	size_t bytes_read = file_handle->file_read_callback(file_handle->handle, (void*) buffer, (size_t) btr, timeout);
	*br = (uint32_t) bytes_read;
	if (bytes_read == 0)
		return data_stream_file_error;
	return data_stream_ok;
}

static data_stream_return_t data_stream_message_validate(const data_stream_message_struct* message) {
	if (memcmp(&(message->header), &(data_stream_header_template), sizeof(data_stream_header_t)) != 0)
		return data_stream_error_invalid_message;
	if (message->metadata.operation != data_stream_operation_read && message->metadata.operation != data_stream_operation_write && message->metadata.operation != data_stream_operation_response)
		return data_stream_error_invalid_message;
#ifdef data_stream_use_checksum
	if ( CRC_xor((const uint8_t*) (&message->metadata), sizeof(data_stream_metadata_t)) != message->checksum)
		return data_stream_error;
#endif
	return data_stream_ok;
}
static data_stream_return_t data_stream_message_init(data_stream_message_struct* message, data_stream_operation_t operation, data_stream_id_t stream_id,uint8_t data_size) {

  message->header=data_stream_header_template;

	message->metadata.operation = operation;
	message->metadata.stream_id = stream_id;
	message->metadata.data_size = data_size;
	#ifdef data_stream_use_checksum
	message->checksum =  CRC_xor((const uint8_t*) (&message->metadata), sizeof(data_stream_metadata_t));
	#endif
	return data_stream_ok;
}

static data_stream_return_t data_stream_compare_stream_metadata(const data_stream_metadata_t* metadata_1, const data_stream_metadata_t* metadata_2) {
	if (metadata_1->stream_id == metadata_2->stream_id && metadata_1->operation == metadata_2->operation && metadata_1->data_size==metadata_2->data_size)
		return data_stream_ok;
	else
		return data_stream_wrong_stream_params;
}

data_stream_return_t data_stream_server_init(data_stream_server_instance* stream, data_stream_data_read_callback_t data_read_callback, data_stream_data_write_callback_t data_write_callback, file_stream_read_callback_t file_read_callback, file_stream_write_callback_t file_write_callback, void* file_handle, get_timestamp_callback_t get_timestamp_fcn, void* timestamp_source, void* user_data) {

	if (stream == NULL)
		return data_stream_error;
	stream->application_data_read_callback = data_read_callback;
	stream->application_data_write_callback = data_write_callback;

	if (file_read_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_read_callback = file_read_callback;
	if (file_write_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_write_callback = file_write_callback;
	stream->file_stream.handle = file_handle;

    #ifdef data_stream_use_timestamp
	stream->get_timestamp_fcn = get_timestamp_fcn;
	stream->timestamp_source = timestamp_source;
    #endif 
	stream->user_data = user_data;

	stream->state = data_stream_state_ready;
  stream->message=(data_stream_message_struct){0};
	stream->data_struct=(data_stream_data_t){0};
	return data_stream_ok;
}

data_stream_return_t data_stream_client_init(data_stream_client_instance* stream,  file_stream_read_callback_t file_read_callback, file_stream_write_callback_t file_write_callback, void* file_handle,  get_timestamp_callback_t get_timestamp_fcn, void* timestamp_source) {

	if (stream == NULL)
		return data_stream_error;
	

	if (file_read_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_read_callback = file_read_callback;
	if (file_write_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_write_callback = file_write_callback;
	stream->file_stream.handle = file_handle;

  stream->get_timestamp_fcn = get_timestamp_fcn;
	stream->timestamp_source = timestamp_source;

	stream->state = data_stream_state_ready;
	stream->message=(data_stream_message_struct){0};
	stream->data_struct=(data_stream_data_t){0};

	return data_stream_ok;
}

data_stream_return_t data_stream_client_write_stream(data_stream_client_instance* stream, data_stream_id_t stream_id,const void* data_ptr, uint8_t data_size) {
	if (data_size > sizeof(stream->data_struct.data)) return data_stream_error;
	
	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;

	data_stream_message_init(&stream->message, data_stream_operation_write, stream_id, data_size);

	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}

#ifdef data_stream_use_timestamp
	if (stream->get_timestamp_fcn != NULL)
		stream->data_struct.timestamp = stream->get_timestamp_fcn(stream->timestamp_source);
	else
		stream->data_struct.timestamp = 0;
#endif

	memcpy((void*)stream->data_struct.data, data_ptr, (size_t)data_size);
    #ifdef data_stream_use_checksum
	stream->data_struct.checksum=CRC_xor((const uint8_t*) stream->data_struct.data, (size_t)data_size);
	#endif

	bytes_written = 0;
	uint32_t bytes_to_write = data_size;
#ifdef data_stream_use_checksum
  bytes_to_write += sizeof(stream->data_struct.checksum);
#endif
#ifdef data_stream_use_timestamp
  bytes_to_write += sizeof(stream->data_struct.timestamp);
#endif
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->data_struct), bytes_to_write, &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != bytes_to_write) {
		returnval = data_stream_file_error;
		goto terminate;
	}

	uint32_t bytes_read = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_read, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_read != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message);
	if (returnval != data_stream_ok) {
		goto terminate;
	}
	
	data_stream_metadata_t metadata_ref = { 0 };
	metadata_ref.operation = data_stream_operation_response;
	metadata_ref.stream_id = stream_id;
	metadata_ref.data_size = data_size;

	returnval = data_stream_compare_stream_metadata(&metadata_ref, &stream->message.metadata);
	if (returnval != data_stream_ok) {
		goto terminate;
	}

	terminate: stream->state = data_stream_state_ready;
	return returnval;

}

data_stream_return_t data_stream_client_read_stream(data_stream_client_instance* stream, data_stream_id_t stream_id, void* data_ptr,uint8_t data_size) {

if (data_size > sizeof(stream->data_struct.data)) return data_stream_error;
	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;
	data_stream_message_init(&stream->message, data_stream_operation_read, stream_id, data_size);

	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}

	uint32_t bytes_read = 0;
	uint32_t bytes_to_read = data_size;
#ifdef data_stream_use_checksum
  bytes_to_read += sizeof(stream->data_struct.checksum);
#endif
#ifdef data_stream_use_timestamp
  bytes_to_read += sizeof(stream->data_struct.timestamp);
#endif
	
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->data_struct), bytes_to_read, &bytes_read, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_read != bytes_to_read) {
		returnval = data_stream_file_error;
		goto terminate;
	}
    #ifdef data_stream_use_checksum
	if(stream->data_struct.checksum!=CRC_xor((const uint8_t*) stream->data_struct.data, (size_t)data_size))
	{
		returnval=data_stream_error;
		goto terminate;
	}
    #endif
	memcpy(data_ptr,(const void*)stream->data_struct.data, (size_t)data_size);
	
	
	data_stream_message_init(&stream->message, data_stream_operation_response, stream_id, data_size);
    
	bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}

	terminate: stream->state = data_stream_state_ready;
	return returnval;

}

data_stream_return_t data_stream_server(data_stream_server_instance* stream) {

	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;

	uint32_t bytes_read = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_read, 0xFFFFFFFF);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_read != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message);
	if (returnval != data_stream_ok)
		goto terminate;

	data_stream_metadata_t metadata_rec=stream->message.metadata;

	
	if (metadata_rec.operation == data_stream_operation_read) {
        returnval = stream->application_data_read_callback(metadata_rec.stream_id, metadata_rec.data_size, stream->data_struct.data, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}

        #ifdef data_stream_use_checksum
		stream->data_struct.checksum=CRC_xor((const uint8_t*) stream->data_struct.data, (size_t)metadata_rec.data_size);
        #endif
#ifdef data_stream_use_timestamp
	if (stream->get_timestamp_fcn != NULL)
		stream->data_struct.timestamp = stream->get_timestamp_fcn(stream->timestamp_source);
	else
		stream->data_struct.timestamp = 0;
#endif

	uint32_t bytes_written = 0;
	uint32_t bytes_to_write = metadata_rec.data_size;
#ifdef data_stream_use_checksum
  bytes_to_write += sizeof(stream->data_struct.checksum);
#endif
#ifdef data_stream_use_timestamp
  bytes_to_write += sizeof(stream->data_struct.timestamp);
#endif
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->data_struct), bytes_to_write, &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != bytes_to_write) {
		returnval = data_stream_file_error;
		goto terminate;
	}


	bytes_read = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_read, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_read != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message);
	if (returnval != data_stream_ok) {
		goto terminate;
	}

	data_stream_metadata_t metadata_ref = { 0 };
	metadata_ref.operation = data_stream_operation_response;
	metadata_ref.stream_id = metadata_rec.stream_id;
	metadata_ref.data_size = metadata_rec.data_size;
	
	returnval = data_stream_compare_stream_metadata(&metadata_ref, &stream->message.metadata);
	if (returnval != data_stream_ok) {
		goto terminate;
	}

	} 
	
	else if (metadata_rec.operation == data_stream_operation_write) {

		uint32_t bytes_read = 0;
	uint32_t bytes_to_read = metadata_rec.data_size;
#ifdef data_stream_use_checksum
  bytes_to_read += sizeof(stream->data_struct.checksum);
#endif
#ifdef data_stream_use_timestamp
  bytes_to_read += sizeof(stream->data_struct.timestamp);
#endif

	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->data_struct), bytes_to_read, &bytes_read, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_read != bytes_to_read) {
		returnval = data_stream_file_error;
		goto terminate;
	}

    #ifdef data_stream_use_checksum    
	if(stream->data_struct.checksum!=CRC_xor((const uint8_t*) stream->data_struct.data, (size_t)metadata_rec.data_size))
	{
		returnval=data_stream_error;
		goto terminate;
	}
    #endif

		returnval = stream->application_data_write_callback(metadata_rec.stream_id, metadata_rec.data_size, stream->data_struct.data, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}

		data_stream_message_init(&stream->message, data_stream_operation_response, metadata_rec.stream_id,metadata_rec.data_size);

	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (const uint8_t*) (&stream->message), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}

	}

   
	terminate: stream->state = data_stream_state_ready;
	return returnval;
}

#ifdef  data_stream_use_checksum

static uint8_t CRC_xor(const uint8_t* ptr, size_t length) {
	uint8_t CRC = 0;
	for (size_t i = 0; i < length; i++) {
		CRC ^= ptr[i];
	}
	return CRC;
}


#endif
