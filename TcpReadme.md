# Notes for TCP feature

Reference pyMultiWii [tcp-feature](https://github.com/wil3/pyMultiWii/tree/feature-tcp) GitHub 

## MultiWii : takes in *fc_address*

​	parses *fc_address* 

​	if it is TCP, sets `self.channel` as `MultiwiiTCPChannel(host, port)`

### MultiwiiTCPChannel

- `buffer_size  = 1024`

- init

  ```python
  def __init__(self, ipaddr, port):
      self.ipaddr = ipaddr
      self.port = port
      self.data_recv = None
  ```

  

- connect

  ```python
  self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  self.sock.connect((self.ipaddr, self.port))
  ```

- close

  ```python
  def close(self):
      if not self.sock:
  		raise Exception("Cannot close, socket never created")
  	self.s.close()
  ```

  

- write

  ```python
  def write(self, message):
      if not self.sock:
      	raise Exception("Cannot write, socket never created")
  	self.sock.send(message)
  ```

- read

  ```python
  def read(self):
  	packet =  self.sock.recv(MultiwiiTCPChannel.BUFFER_SIZE)
  	payload = bytes(packet)
  	(direction, datalength, command, data, crc) = self._parse_payload(payload)
      if not self.is_checksum_valid(payload[3 : 3 + 1 + 1 + datalength], crc):
              raise Exception("Checksum not valid!")
  ```

  