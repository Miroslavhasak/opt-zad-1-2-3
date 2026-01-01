
#ifndef DATA_STREAM_H_
#define DATA_STREAM_H_

#include <stdint.h>
#include <stddef.h>

#define data_stream_use_timestamp ///<configuration switch - adds timestamp information to message
#define data_stream_use_checksum  ///<configuration switch - adds checksum information to message


#define data_stream_header_length 1 ///<length of message header (bytes)
#define data_stream_buffer_size 64 ///<length of the data buffer


#define data_stream_read_timeout 1000 ///<file stream read timeout
#define data_stream_write_timeout 1000 ///<file stream write timeout

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	data_stream_ok = 0,
	data_stream_error, ///<general error, checksum error
	data_stream_error_invalid_message, ///<message verification failed
	data_stream_file_error, ///<file transfer (read/write) failed
	data_stream_error_busy, ///<data stream is busy - operation pending
	data_stream_wrong_stream_params ///< messages metadata not match, stream id does not exist
} data_stream_return_t;


typedef enum {
	data_stream_state_ready = 0, ///< data stream ready for API calls
	data_stream_state_busy = 1 ///< data stream operation pending (busy), e.g. waiting for file transfer complete
} data_stream_status_t;



typedef struct {uint8_t header_arr [data_stream_header_length];} data_stream_header_t; ///< message header byte array typedef
typedef uint8_t data_stream_id_t; ///<stream index number typedef (byte) (0-255)
typedef uint8_t data_stream_operation_t; ///<message operation type typedef (byte)
typedef float data_stream_timestamp_t; ///<message transferred timestamp type (float)

enum {
	data_stream_operation_unknown = 0, ///<not assigned operation type (default)
	data_stream_operation_read, ///<data read operation message
	data_stream_operation_write, ///<data write operation message
	data_stream_operation_response ///<response (answer) operation message
};


typedef struct {
	data_stream_id_t stream_id :4; ///<message stream index number
	data_stream_operation_t operation :4; ///<message operation type information
	uint8_t data_size; ///<size of data
} data_stream_metadata_t;


typedef struct {
	#ifdef data_stream_use_timestamp
	data_stream_timestamp_t timestamp; ///<Message timestamp information (optional)
	#endif
#ifdef data_stream_use_checksum
	uint8_t checksum; ///<Message checksum information (optional)
#endif
	uint8_t data [data_stream_buffer_size] ; 
} data_stream_data_t;



typedef struct {
	data_stream_header_t header; ///<Message header data - leading constant byte array
	data_stream_metadata_t metadata; ///<Message metadata information - bitfield structure
#ifdef data_stream_use_checksum
	uint8_t checksum; ///<Message checksum information (optional)
#endif
}__attribute__((packed)) data_stream_message_struct;


typedef data_stream_return_t (*data_stream_data_write_callback_t)(data_stream_id_t,uint8_t,const void*,void*);
typedef data_stream_return_t (*data_stream_data_read_callback_t)(data_stream_id_t,uint8_t, void*,void*);
typedef data_stream_timestamp_t (*get_timestamp_callback_t)(void*);


typedef size_t (*file_stream_write_callback_t)(void*, const void*, size_t, uint32_t);
typedef size_t (*file_stream_read_callback_t)(void*, void*, size_t, uint32_t);


typedef struct {
	void* handle; ///<pointer to file handle
	file_stream_write_callback_t file_write_callback; ///< data write function pointer
	file_stream_read_callback_t file_read_callback; ///< data read function pointer
} data_stream_file_stream_generic;

typedef struct {
	data_stream_message_struct message; 
	data_stream_data_t data_struct;
	data_stream_status_t state;	///<Instance locking state
	data_stream_file_stream_generic file_stream; ///<Transfer medium file stream handle
  void* timestamp_source; ///<Timestamp source handle (optional)
	get_timestamp_callback_t get_timestamp_fcn; ///<Registered timestamp callback function
} data_stream_client_instance;


 typedef struct {
	data_stream_message_struct message; 
	data_stream_data_t data_struct;
	data_stream_status_t state;	///<Instance locking state
	data_stream_file_stream_generic file_stream; ///<Transfer medium file stream handle
	void* timestamp_source; ///<Timestamp source handle (optional)
	get_timestamp_callback_t get_timestamp_fcn; ///<Registered timestamp callback function
	void* user_data; ///<Pointer to user data (optional)
  data_stream_data_read_callback_t application_data_read_callback; ///<Registered data read callback function(required for server)
	data_stream_data_write_callback_t application_data_write_callback; ///<Registered data write callback function(required for server)
} data_stream_server_instance;

data_stream_return_t data_stream_server_init(data_stream_server_instance*, data_stream_data_read_callback_t, data_stream_data_write_callback_t, file_stream_read_callback_t, file_stream_write_callback_t, void*, get_timestamp_callback_t, void*,void*);
data_stream_return_t data_stream_client_init(data_stream_client_instance*, file_stream_read_callback_t, file_stream_write_callback_t,void*, get_timestamp_callback_t, void*);

data_stream_return_t data_stream_client_write_stream(data_stream_client_instance*, data_stream_id_t,const void*,uint8_t);
data_stream_return_t data_stream_client_read_stream(data_stream_client_instance*, data_stream_id_t, void*,uint8_t);
data_stream_return_t data_stream_server(data_stream_server_instance*);

#ifdef data_stream_use_timestamp
static inline data_stream_timestamp_t data_stream_get_received_timestamp(const data_stream_client_instance* stream) {
	return stream->data_struct.timestamp;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* DATA_STREAM_H_ */
