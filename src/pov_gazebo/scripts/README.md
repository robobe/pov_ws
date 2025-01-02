

## Service

### call

```
gz service --info -s /world/runway/set_pose
Service providers [Address, Request Message Type, Response Message Type]:
  tcp://172.17.0.1:45395, gz.msgs.Pose, gz.msgs.Boolean
```

```bash
# 
gz service -s /world/default/set_pose \
--reqtype gz.msgs.Pose \
--reptype gz.msgs.Boolean \
--timeout 300 \
--req \
'name: "simple_box", position: {x:5.0, y:1.0 z: 0.0}'
```