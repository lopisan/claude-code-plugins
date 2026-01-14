---
description: List and inspect ROS services, view service types, and call services
argument-hint: [service-name] | call <service> <args>
allowed-tools: Bash
model: haiku
---

Manage ROS services. The user may optionally provide a service name or call command as $ARGUMENTS.

## If no arguments (list all services):

```bash
rosservice list 2>&1
```

## If service name provided (starts with /):

### Get service info:
```bash
rosservice info "$ARGUMENTS" 2>&1
```

### Get service type:
```bash
rosservice type "$ARGUMENTS" 2>&1
```

### Show service definition:
```bash
SRV_TYPE=$(rosservice type "$ARGUMENTS" 2>/dev/null)
if [ -n "$SRV_TYPE" ]; then
  rossrv show $SRV_TYPE 2>&1
fi
```

## If "call" command provided:

Parse the service name and arguments from $ARGUMENTS, then:

```bash
# Example: rosservice call /service_name "data: true"
rosservice call <service_name> <args> 2>&1
```

## Output Format

### For service list:

| Service | Type |
|---------|------|
| /rosout/get_loggers | roscpp/GetLoggers |
| /rosout/set_logger_level | roscpp/SetLoggerLevel |

### For specific service:

**Service:** /your_service
**Type:** std_srvs/SetBool
**Node:** /your_node

**Request:**
```
bool data
```

**Response:**
```
bool success
string message
```

### For service call:

**Called:** /your_service
**Request:** `{data: true}`
**Response:**
```
success: True
message: "Service executed successfully"
```

### Common Operations

- `rosservice call /service '{}'` - Call service with empty request
- `rosservice call /set_bool "data: true"` - Call SetBool service
- `rosservice find std_srvs/Empty` - Find services of type
