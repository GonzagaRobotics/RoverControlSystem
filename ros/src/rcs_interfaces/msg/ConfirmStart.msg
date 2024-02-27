# How long between each heartbeat the client sends to the rover (seconds)
float32 heartbeat_interval

# How long between heartbeats before it's considered a timeout (seconds)
float32 heartbeat_timeout

# How many timeouts before the connection is considered lost
uint32 heartbeat_timeout_limit