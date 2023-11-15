# Technical Specifications

## :warning: This is experimental! All contents are subject to frequent change. :warning:

This document describes the technical specifications for RCS, including the topics and messages that are required by RCS.

## Naming Conventions

The term **entity** refers to either a component or a system.

All entities must have a unique name that RCS can use to identify them. This name is equivalent to the entity's ROS node name.

All topics and actions that an entity uses must be prefixed with the entity's name. For example, an component named `camera` would have a topic named `camera/TOPIC_NAME`.

_TODO: What if a component has multiple nodes?_

## Bandwidth Reduction

Can **optionally** be implemented by

- Components
- Systems

The bandwidth reduction is a subscriber named `ENTITY_NAME/bandwidth_reduction`. The entity can subscribe to the publisher `rcs_core/bandwidth_reduction`. The default level of bandwidth reduction is 0. Any message received on this topic overrides the previous level, and will not be changed until another message is received.

Message Interface: `Bandwidth Reduction` in [Message Interfaces](#message-interfaces)

## Heartbeat

**Must** to be implemented by

- Components
- Remote Systems
- RCS Core

**Cannot** be implemented by

- Intra-Rover Systems

The heartbeat is a publisher named `ENTITY_NAME/heartbeat/pub`.

On a regular interval, the entity must publish a message to the subscriber `rcs_core/heartbeat/sub`.

The RCS Core must also publish a message to each remote system on the subscriber `REMOTE_SYSTEM_NAME/heartbeat/sub`, using the publisher `rcs_core/heartbeat/pub`.

Effectively, each component is sending a heartbeat to the RCS Core, while the RCS Core and each remote system are sending heartbeats to each other.

Message Interface: `Heartbeat` in [Message Interfaces](#message-interfaces)

_TODO: How long is a "regular interval", 5secs? A grace period is probably needed._

## Component Status Send

**Must** be implemented by

- RCS Core

**Cannot** be implemented by

- Components
- Systems

The component status send is a publisher named `rcs_core/component_status`.

Every time a component is added or removed, the RCS Core must publish a message to each remote system on the subscriber `REMOTE_SYSTEM_NAME/component_status`. RCS Core must also publish this message every time the status of a component changes.

Message Interface: `Component Status` in [Message Interfaces](#message-interfaces)

## Component Status Receive

**Must** be implemented by

- Remote Systems

**Cannot** be implemented by

- Components
- Intra-Rover Systems

The component status receive is a subscriber named `REMOTE_SYSTEM_NAME/component_status`. Whenever a message is received on this topic, the remote system must update its internal list of components and their statuses.

Message Interface: `Component Status` in [Message Interfaces](#message-interfaces)

## Remote System Init Send

**Must** be implemented by

- Remote Systems

**Cannot** be implemented by

- Components
- Intra-Rover Systems

The remote system init send is a publisher named `REMOTE_SYSTEM_NAME/init`.

When a remote system is connected, it must publish a message to the subscriber `rcs_core/init`.

Message Interface: `Remote System Init` in [Message Interfaces](#message-interfaces)

## Remote System Init Receive

**Must** be implemented by

- RCS Core

**Cannot** be implemented by

- Components
- Systems

The remote system init receive is a subscriber named `rcs_core/init`. Whenever a message is received on this topic, the RCS Core must update its internal list of remote systems. It will then send component status messages to the remote system.

Message Interface: `Remote System Init` in [Message Interfaces](#message-interfaces)

## Message Interfaces

### Heartbeat

```
# Timestamp (frame_id is not used)
std_msgs/Header header

# The name of the component that is sending the heartbeat
string node_name
```

### Bandwidth Reduction

```
# Level of bandwidth reduction (0-3)
uint8 level
```

### Component Status

```
# Both arrays are parallel

# The names of all components in RCS
string[] component_names

# The status of each component (can be "active" or "failed")
string[] component_statuses
```

### Remote System Init

```
# The name of the remote system
string name
```

## RCS Startup Sequence

- RCS Initialization
  1. RCS Core initializes its internal list of components and remote systems
  2. RCS Core starts the heartbeat timer
- Component Initialization (for each component)
  1. RCS Core waits for a component to send a heartbeat
  2. RCS Core updates component status
- Remote System Initialization (for each remote system)
  1. RCS Core waits for a remote system to send a heartbeat
  2. RCS Core begins sending heartbeats back to remote system
  3. RCS Core waits for remote system to send init message
  4. RCS Core sends component status to remote system
