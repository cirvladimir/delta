protoc -I . --grpc_out=../camera_server --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` camera_service.proto
protoc -I . --cpp_out=../camera_server camera_service.proto
python3 -m grpc_tools.protoc -I . --python_out=../python --grpc_python_out=../python camera_service.proto
