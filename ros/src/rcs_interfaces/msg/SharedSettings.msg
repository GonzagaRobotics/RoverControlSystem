# How long between each heartbeat the client sends to the rover (seconds)
float32 heartbeat_interval

# How long between heartbeats before it's considered a timeout (seconds)
float32 heartbeat_timeout

# When this many timeouts occur, the connection is considered lost
uint32 heartbeat_timeout_limit