xxx
## TVCLOC Client

This package get data from TVC LOC via modbus protocol, publish to rostopic and receive from rostopic (odom/encoder) then transfer to TvcLoc.

## Dependencies

### libmodbus

1. Download and install following instructions from [this link](https://libmodbus.org/download/)

## Usage

### Configuration

[`config.yaml`](configs/config.yaml)

``` yaml
spin_rate: 100

tcp: true
ip: "192.168.100.17"
port: 1502

# msg_type: 1               # 1: odometry, 2: Encoder
# msg_type_version: 5       # 4: velocity-based. 5: position-based
# odom_topic: "odom"

msg_type: 2
encoder_topic: "encoder"
```

- `tcp`:
  - `true` (default): Use modbus tcp
  - `false`: Use modbus rtu (RS232/RS485)

- If `tcp: true`:
  - `ip`: IP of TvcLoc PC
  - `port`: tcp port


- `msg_type`:
  - 1: transfer odometry data
  - 2: transfer encoder data
- `msg_type_version`: For `msg_type = 1`.
  - 4: positioning-based odometry
  - 5: velocity-based odometry

- `odom_topic`: (`msg_type = 1`). Define name of topic that publish odometry
- `encoder_topic`: (`msg_type = 2`) Define name of topic that publish encoder topic. Encoder must be published by type [`tvcloc_client::Encoder`](msg/Encoder.msg)

### TvcLoc output

- Topic: `/loc_result`, type: [`tvcloc_client::LocResult.msg`](msg/LocResult.msg)

```
Header              header

geometry_msgs/Pose  pose
int8               confident
int8               matching_score
int8               loc_status
```

### Odometry input

- Subscribe to topic was configured in `odom_topic`
- Positioning-based odometry:
  ```
  msg_type: 1
  msg_type_version: 4
  ```
  - Get velocity from `<odom_topic>/twist` to transfer to TvcLoc
- Velocity-based odometry:
  ```
  msg_type: 1
  msg_type_version: 5
  ```
  - Get velocity from `<odom_topic>/pose` to transfer to TvcLoc

### Encoders input
- Subscribe to topic was configured in `encoder_topic`
- Configuration: `msg_type = 2`.


**NOTE** Must be configured accordingly on TvcLoc