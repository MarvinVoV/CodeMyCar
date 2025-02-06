
## 协议分层设计
+---------------------+
| UART Frame Header   | protocol_header_t
+---------------------+
| Application Payload | 自定义指令结构
+---------------------+
| Checksum & Frame End|
+---------------------+
