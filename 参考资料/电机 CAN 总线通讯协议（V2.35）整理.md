# 电机 CAN 总线通讯协议（V2.35）整理

我将根据提供的电机CAN总线通讯协议文档内容，整理生成结构清晰的Word文档。文档将包含免责声明、CAN总线参数、单电机命令（含各命令的格式、数据域说明、驱动回复等），并保持原有表格格式和参数细节完整。

由于无法直接生成Word文件，我将提供完整的文档内容（可直接复制粘贴到Word中使用），格式采用Markdown规范，便于在Word中调整排版：

# 电机CAN总线通讯协议（V2.35）

## 免责声明

感谢您购买本公司电机驱动一体控制系统。在使用之前，请仔细阅读本声明，一旦使用，即被视为对本声明全部内容的认可和接受。请严格遵守产品手册、控制协议和相关的法律法规、政策、准则安装和使用该产品。在使用产品过程中，用户承诺对自己的行为及因此而产生的所有后果负责。因用户不当使用、安装、改装造成的任何损失，本公司将不承担法律责任。

## 一、CAN总线参数

### 总线接口

CAN

### 波特率（常规模式，单电机命令）

- 1Mbps（默认）

- 500kbps

- 250kbps

- 125kbps

- 100kbps

### 波特率（广播模式，多电机命令）

- 1Mbps

- 500kbps

## 二、单电机命令

同一总线上共可以挂载多达32（视总线负载情况而定）个驱动，为了防止总线冲突，每个驱动需要设置不同的ID。

主控向总线发送单电机命令，对应ID的电机在收到命令后执行，并在一段时间后（0.25ms内）向主控发送回复。

### 命令报文和回复报文格式

- 标识符：0x140 + ID（1~32）

- 帧格式：数据帧

- 帧类型：标准帧

- DLC：8字节

### 1. 读取电机状态1和错误标志命令（1帧）

该命令读取当前电机的温度、电压和错误状态标志

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9A|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据包含了以下参数：

1. 电机温度temperature（int8_t类型，单位1℃/LSB）。

2. 母线电压voltage（uint16_t类型，单位0.01V/LSB）。

3. 母线电流current（uint16_t类型，单位0.01A/LSB）。

4. 电机状态motorState（为uint8_t类型，各个位代表不同的电机状态）

5. 错误标志errorState（为uint8_t类型，各个位代表不同的电机错误状态）

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9A|
|DATA[1]|电机温度|DATA[1] = *(uint8_t *)(&temperature)|
|DATA[2]|母线电压低字节|DATA[2] = *(uint8_t *)(&voltage)|
|DATA[3]|母线电压高字节|DATA[3] = *((uint8_t *)(&voltage)+1)|
|DATA[4]|母线电流低字节|DATA[4] = *(uint8_t *)(&current)|
|DATA[5]|母线电流高字节|DATA[5] = *((uint8_t *)(&current)+1)|
|DATA[6]|电机状态字节|DATA[6] = motorState|
|DATA[7]|错误状态字节|DATA[7] = errorState|
##### 备注

1. motorState = 0x00 电机处于开启状态；motorState = 0x10 电机处于关闭状态。

2. errorState各个位具体状态表如下

|errorState位|状态说明|0|1|
|---|---|---|---|
|0|电压状态|电压正常|低压保护|
|1|无效|-|-|
|2|无效|-|-|
|3|温度状态|温度正常|过温保护|
|4|无效|-|-|
|5|无效|-|-|
|6|无效|-|-|
|7|无效|-|-|
### 2. 清除电机错误标志命令（1帧）

该命令清除当前电机的错误状态，电机收到后返回

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9B|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机。回复数据和读取电机状态1和错误标志命令相同（仅命令字节DATA[0]不同，这里为0x9B）

##### 备注

1. 电机状态没有恢复正常时，错误标志无法清除。

### 3. 读取电机状态2命令（1帧）

该命令读取当前电机的温度、电机转矩电流（K、TS）/电机输出功率（L）、转速、编码器位置。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9C|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 电机温度temperature（int8_t类型，1℃/LSB）。

2. K、TS电机的转矩电流值iq或L电机的输出功率值power，int16_t类型。TS电机iq分辨率为（66/4096 A）/ LSB；K电机iq分辨率为（33/4096 A）/ LSB。L电机power范围-1000~1000。

3. 电机转速speed（int16_t类型，1dps/LSB）。

4. 编码器位置值encoder（uint16_t类型，14bit编码器的数值范围016383，16bit编码器的数值范围065535）。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9C|
|DATA[1]|电机温度|DATA[1] = *(uint8_t *)(&temperature)|
|DATA[2]|转矩电流低字节/输出功率低字节（L系列）|DATA[2] = *(uint8_t *)(&iq) / DATA[2] = *(uint8_t *)(&power)|
|DATA[3]|转矩电流高字节/输出功率高字节（L系列）|DATA[3] = *((uint8_t *)(&iq)+1) / DATA[3] = *((uint8_t *)(&power)+1)|
|DATA[4]|电机速度低字节|DATA[4] = *(uint8_t *)(&speed)|
|DATA[5]|电机速度高字节|DATA[5] = *((uint8_t *)(&speed)+1)|
|DATA[6]|编码器位置低字节|DATA[6] = *(uint8_t *)(&encoder)|
|DATA[7]|编码器位置高字节|DATA[7] = *((uint8_t *)(&encoder)+1)|
### 4. 读取电机状态3命令（1帧）

由于L电机没有相电流采样，该命令在L电机上无作用。该命令读取当前电机的温度和3相电流数据

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9D|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据包含了以下数据：

1. 电机温度temperature（int8_t类型，1℃/LSB）

2. 相电流数据iA、iB、iC，数据类型为int16_t类型，TS电机相电流分辨率为（66/4096 A）/ LSB；K电机相电流分辨率为（33/4096 A）/ LSB。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x9D|
|DATA[1]|电机温度|DATA[1] = *(uint8_t *)(&temperature)|
|DATA[2]|A相电流低字节|DATA[2] = *(uint8_t *)(&iA)|
|DATA[3]|A相电流高字节|DATA[3] = *((uint8_t *)(&iA)+1)|
|DATA[4]|B相电流低字节|DATA[4] = *(uint8_t *)(&iB)|
|DATA[5]|B相电流高字节|DATA[5] = *((uint8_t *)(&iB)+1)|
|DATA[6]|C相电流低字节|DATA[6] = *(uint8_t *)(&iC)|
|DATA[7]|C相电流高字节|DATA[7] = *((uint8_t *)(&iC)+1)|
### 5. 电机关闭命令

将电机从开启状态（上电后默认状态）切换到关闭状态，清除电机转动圈数及之前接收的控制指令，LED由常亮转为慢闪。此时电机仍然可以回复控制命令，但不会执行动作。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x80|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复

和主机发送相同。

### 6. 电机运行命令

将电机从关闭状态切换到开启状态，LED由慢闪转为常亮。此时再发送控制指令即可控制电机动作。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x88|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复

和主机发送相同。

### 7. 电机停止命令

停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x81|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

和主机发送相同。

### 8. 抱闸器控制命令

控制抱闸器的开合，或者读取当前抱闸器的状态。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x8C|
|DATA[1]|抱闸器状态控制和读取字节|0x00：抱闸器断电，刹车启动；0x01：抱闸器通电，刹车释放；0x10：读取抱闸器状态|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x8C|
|DATA[1]|抱闸器状态字节|0x00：抱闸器处于断电状态，刹车启动；0x01：抱闸器处于通电状态，刹车释放|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
### 9. 开环控制命令（该命令仅在L电机上实现，其他电机无效）

主机发送该命令以控制输出到电机的开环电压，控制值powerControl为int16_t类型，数值范围-850~850，（电机电流和扭矩因电机而异）。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA0|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|开环控制值低字节|DATA[4] = *(uint8_t *)(&powerControl)|
|DATA[5]|开环控制值高字节|DATA[5] = *((uint8_t *)(&powerControl)+1)|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 备注

1. 该命令中的控制值powerControl不受上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 电机温度temperature，int8_t类型，1℃/LSB。

2. 电机输出功率值power，int16_t类型，范围-850~850。

3. 电机转速speed，int16_t类型，1dps/LSB。

4. 编码器位置值encoder，uint16_t类型，15bit编码器的数值范围032767；18bit编码器的数值范围065535（保留高16bit，省略低2bit）。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA0|
|DATA[1]|电机温度|DATA[1] = *(uint8_t *)(&temperature)|
|DATA[2]|转矩电流低字节|DATA[2] = *(uint8_t *)(&power)|
|DATA[3]|转矩电流高字节|DATA[3] = *((uint8_t *)(&power)+1)|
|DATA[4]|电机速度低字节|DATA[4] = *(uint8_t *)(&speed)|
|DATA[5]|电机速度高字节|DATA[5] = *((uint8_t *)(&speed)+1)|
|DATA[6]|编码器位置低字节|DATA[6] = *(uint8_t *)(&encoder)|
|DATA[7]|编码器位置高字节|DATA[7] = *((uint8_t *)(&encoder)+1)|
### 10. 转矩闭环控制命令（该命令仅在K、ZH、TS电机上实现）

主机发送该命令以控制电机的转矩电流输出，控制值iqControl为int16_t类型，数值范围-20482048，对应K电机实际转矩电流范围-16.5A16.5A，对应TS电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电机而异。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA1|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|转矩电流控制值低字节|DATA[4] = *(uint8_t *)(&iqControl)|
|DATA[5]|转矩电流控制值高字节|DATA[5] = *((uint8_t *)(&iqControl)+1)|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 备注

1. 该命令中的控制值iqControl不受上位机中的Max Torque Current值限制。

#### 驱动回复

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 电机温度temperature，int8_t类型，1℃/LSB。

2. 电机的转矩电流值iq，int16_t类型，范围-20482048，对应K电机实际转矩电流范围-16.5A16.5A，对应TS电机实际转矩电流范围-33A~33A。

3. 电机转速speed，int16_t类型，1dps/LSB。

4. 编码器位置值encoder，uint16_t类型，14bit编码器的数值范围016383；15bit编码器的数值范围032767；18bit编码器的数值范围0~65535（保留高位16bit，省略低位2bit）。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA1|
|DATA[1]|电机温度|DATA[1] = *(uint8_t *)(&temperature)|
|DATA[2]|转矩电流低字节|DATA[2] = *(uint8_t *)(&iq)|
|DATA[3]|转矩电流高字节|DATA[3] = *((uint8_t *)(&iq)+1)|
|DATA[4]|电机速度低字节|DATA[4] = *(uint8_t *)(&speed)|
|DATA[5]|电机速度高字节|DATA[5] = *((uint8_t *)(&speed)+1)|
|DATA[6]|编码器位置低字节|DATA[6] = *(uint8_t *)(&encoder)|
|DATA[7]|编码器位置高字节|DATA[7] = *((uint8_t *)(&encoder)+1)|
### 11. 速度闭环控制命令（1帧）

主机发送该命令以控制电机的速度，控制值speedControl为int32_t类型，对应实际转速为0.01dps/LSB。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA2|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|速度控制低字节|DATA[4] = *(uint8_t *)(&speedControl)|
|DATA[5]|速度控制|DATA[5] = *((uint8_t *)(&speedControl)+1)|
|DATA[6]|速度控制|DATA[6] = *((uint8_t *)(&speedControl)+2)|
|DATA[7]|速度控制高字节|DATA[7] = *((uint8_t *)(&speedControl)+3)|
#### 备注

1. 该命令下电机的speedControl由上位机中的Max Speed值限制。

2. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

3. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA2）；K、ZH、TS电机回复数据和转矩闭环控制命令相同（仅命令字节不同，这里为0xA2）。

### 12. 多圈位置闭环控制命令1（1帧）

主机发送该命令以控制电机的位置（多圈角度），控制值angleControl为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°，电机转动方向由目标位置和当前位置的差值决定。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA3|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|位置控制低字节|DATA[4] = *(uint8_t *)(&angleControl)|
|DATA[5]|位置控制|DATA[5] = *((uint8_t *)(&angleControl)+1)|
|DATA[6]|位置控制|DATA[6] = *((uint8_t *)(&angleControl)+2)|
|DATA[7]|位置控制高字节|DATA[7] = *((uint8_t *)(&angleControl)+3)|
#### 备注

1. 该命令下的控制值angleControl受上位机中的Max Angle值限制。

2. 该命令下电机的最大速度由上位机中的Max Speed值限制。

3. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

4. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机，L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA3）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA3）。

### 13. 多圈位置闭环控制命令2（1帧）

主机发送该命令以控制电机的位置（多圈角度）

1. 控制值angleControl为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°，电机转动方向由目标位置和当前位置的差值决定。

2. 控制值maxSpeed限制了电机转动的最大速度，为uint16_t类型，对应实际转速1dps/LSB，即360代表360dps。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA4|
|DATA[1]|NULL|0x00|
|DATA[2]|速度限制低字节|DATA[2] = *(uint8_t *)(&maxSpeed)|
|DATA[3]|速度限制高字节|DATA[3] = *((uint8_t *)(&maxSpeed)+1)|
|DATA[4]|位置控制低字节|DATA[4] = *(uint8_t *)(&angleControl)|
|DATA[5]|位置控制|DATA[5] = *((uint8_t *)(&angleControl)+1)|
|DATA[6]|位置控制|DATA[6] = *((uint8_t *)(&angleControl)+2)|
|DATA[7]|位置控制高字节|DATA[7] = *((uint8_t *)(&angleControl)+3)|
#### 备注

1. 该命令下的控制值angleControl受上位机中的Max Angle值限制。

2. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

3. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA4）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA4）。

### 14. 单圈位置闭环控制命令1（1帧）

主机发送该命令以控制电机的位置（单圈角度）。

1. 控制值spinDirection设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针

2. 控制值angleControl为uint32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA5|
|DATA[1]|转动方向字节|DATA[1] = spinDirection|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|位置控制字节1（bit0:bit7）|DATA[4] = *(uint8_t *)(&angleControl)|
|DATA[5]|位置控制字节2（bit8:bit15）|DATA[5] = *((uint8_t *)(&angleControl)+1)|
|DATA[6]|位置控制字节3（bit16:bit23）|DATA[6] = *((uint8_t *)(&angleControl)+2)|
|DATA[7]|位置控制字节4（bit24:bit31）|DATA[7] = *((uint8_t *)(&angleControl)+3)|
#### 备注

1. 该命令下电机的最大速度由上位机中的Max Speed值限制。

2. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

3. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA5）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA5）。

### 15. 单圈位置闭环控制命令2（1帧）

主机发送该命令以控制电机的位置（单圈角度）。

1. 控制值spinDirection设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针

2. angleControl为uint32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°。

3. 速度控制值maxSpeed限制了电机转动的最大速度，为uint16_t类型，对应实际转速1dps/LSB，即360代表360dps。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA6|
|DATA[1]|转动方向字节|DATA[1] = spinDirection|
|DATA[2]|速度限制字节1（bit0:bit7）|DATA[2] = *(uint8_t *)(&maxSpeed)|
|DATA[3]|速度限制字节2（bit8:bit15）|DATA[3] = *((uint8_t *)(&maxSpeed)+1)|
|DATA[4]|位置控制字节1（bit0:bit7）|DATA[4] = *(uint8_t *)(&angleControl)|
|DATA[5]|位置控制字节2（bit8:bit15）|DATA[5] = *((uint8_t *)(&angleControl)+1)|
|DATA[6]|位置控制字节3（bit16:bit23）|DATA[6] = *((uint8_t *)(&angleControl)+2)|
|DATA[7]|位置控制字节4（bit24:bit31）|DATA[7] = *((uint8_t *)(&angleControl)+3)|
#### 备注

1. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

2. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA6）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA6）。

### 16. 增量位置闭环控制命令1（1帧）

主机发送该命令以控制电机的位置增量。

控制值angleIncrement为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°，电机的转动方向由该参数的符号决定。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA7|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|位置控制低字节|DATA[4] = *(uint8_t *)(&angleIncrement)|
|DATA[5]|位置控制|DATA[5] = *((uint8_t *)(&angleIncrement)+1)|
|DATA[6]|位置控制|DATA[6] = *((uint8_t *)(&angleIncrement)+2)|
|DATA[7]|位置控制高字节|DATA[7] = *((uint8_t *)(&angleIncrement)+3)|
#### 备注

1. 该命令下电机的最大速度由上位机中的Max Speed值限制。

2. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

3. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA7）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA7）。

### 17. 增量位置闭环控制命令2（1帧）

主机发送该命令以控制电机的位置增量。

1. 控制值angleIncrement为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°，电机转动方向由该参数的符号决定。

2. 控制值maxSpeed限制了电机转动的最大速度，为uint32_t类型，对应实际转速1dps/LSB，即360代表360dps。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0xA8|
|DATA[1]|NULL|0x00|
|DATA[2]|速度限制低字节|DATA[2] = *(uint8_t *)(&maxSpeed)|
|DATA[3]|速度限制高字节|DATA[3] = *((uint8_t *)(&maxSpeed)+1)|
|DATA[4]|位置控制低字节|DATA[4] = *(uint8_t *)(&angleIncrement)|
|DATA[5]|位置控制|DATA[5] = *((uint8_t *)(&angleIncrement)+1)|
|DATA[6]|位置控制|DATA[6] = *((uint8_t *)(&angleIncrement)+2)|
|DATA[7]|位置控制高字节|DATA[7] = *((uint8_t *)(&angleIncrement)+3)|
#### 备注

1. 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。

2. 该控制模式下，K、ZH、TS电机的最大转矩电流由上位机中的Max Torque Current值限制；L电机的最大功率由上位机中的Max Power值限制。

#### 驱动回复（1帧）

电机在收到命令后回复主机。L电机回复数据和开环控制命令相同（仅命令字节不同，这里为0xA8）；K、ZH、TS电机回复数据和力矩闭环控制命令相同（仅命令字节不同，这里为0xA8）。

### 18. 读取PID参数命令（1帧）

主机发送该命令读取当前电机的PID参数

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x30|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

驱动回复数据中包含了各个控制环路的PI参数。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x30|
|DATA[1]|NULL|0x00|
|DATA[2]|位置环P参数|DATA[2] = anglePidKp|
|DATA[3]|位置环I参数|DATA[3] = anglePidKi|
|DATA[4]|速度环P参数|DATA[4] = speedPidKp|
|DATA[5]|速度环I参数|DATA[5] = speedPidKi|
|DATA[6]|转矩环P参数|DATA[6] = iqPidKp|
|DATA[7]|转矩环I参数|DATA[7] = iqPidKi|
### 19. 写入PID参数到RAM命令（1帧）

主机发送该命令写入PID参数到RAM中，断电后写入参数失效

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x31|
|DATA[1]|NULL|0x00|
|DATA[2]|位置环P参数|DATA[2] = anglePidKp|
|DATA[3]|位置环I参数|DATA[3] = anglePidKi|
|DATA[4]|速度环P参数|DATA[4] = speedPidKp|
|DATA[5]|速度环I参数|DATA[5] = speedPidKi|
|DATA[6]|转矩环P参数|DATA[6] = iqPidKp|
|DATA[7]|转矩环I参数|DATA[7] = iqPidKi|
#### 驱动回复（1帧）

电机在收到命令后回复主机，回复命令和接收命令一致

### 20. 写入PID参数到ROM命令（1帧）

主机发送该命令写入PID参数到ROM中，断电仍然有效

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x32|
|DATA[1]|NULL|0x00|
|DATA[2]|位置环P参数|DATA[2] = anglePidKp|
|DATA[3]|位置环I参数|DATA[3] = anglePidKi|
|DATA[4]|速度环P参数|DATA[4] = speedPidKp|
|DATA[5]|速度环I参数|DATA[5] = speedPidKi|
|DATA[6]|转矩环P参数|DATA[6] = iqPidKp|
|DATA[7]|转矩环I参数|DATA[7] = iqPidKi|
#### 驱动回复（1帧）

电机在收到命令后回复主机，回复命令和接收命令一致

### 21. 读取加速度命令（1帧）

主机发送该命令读取当前电机的加速度参数

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x33|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

驱动回复数据中包含了加速度参数。加速度数据Accel为int32_t类型，单位1dps/s

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x33|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|加速度低字节1|DATA[4] = *(uint8_t *)(&Accel)|
|DATA[5]|加速度字节2|DATA[5] = *((uint8_t *)(&Accel)+1)|
|DATA[6]|加速度字节3|DATA[6] = *((uint8_t *)(&Accel)+2)|
|DATA[7]|加速度字节4|DATA[7] = *((uint8_t *)(&Accel)+3)|
### 22. 写入加速度到RAM命令（1帧）

主机发送该命令写入加速度到RAM中，断电后写入参数失效。加速度数据Accel为int32_t类型，单位1dps/s

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x34|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|加速度低字节1|DATA[4] = *(uint8_t *)(&Accel)|
|DATA[5]|加速度字节2|DATA[5] = *((uint8_t *)(&Accel)+1)|
|DATA[6]|加速度字节3|DATA[6] = *((uint8_t *)(&Accel)+2)|
|DATA[7]|加速度字节4|DATA[7] = *((uint8_t *)(&Accel)+3)|
#### 驱动回复（1帧）

电机在收到命令后回复主机，回复命令和接收命令一致

### 23. 读取编码器数据命令（1帧）

主机发送该命令以读取编码器的当前位置

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x90|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 编码器位置encoder（uint16_t类型，14bit编码器的数值范围0~16383），为编码器原始位置减去编码器零偏后的值。

2. 编码器原始位置encoderRaw（uint16_t类型，14bit编码器的数值范围0~16383）。

3. 编码器零偏encoderOffset（uint16_t类型，14bit编码器的数值范围0~16383），该点作为电机角度的0点。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x90|
|DATA[1]|NULL|0x00|
|DATA[2]|编码器位置低字节|DATA[2] = *(uint8_t *)(&encoder)|
|DATA[3]|编码器位置高字节|DATA[3] = *((uint8_t *)(&encoder)+1)|
|DATA[4]|编码器原始位置低字节|DATA[4] = *(uint8_t *)(&encoderRaw)|
|DATA[5]|编码器原始位置高字节|DATA[5] = *((uint8_t *)(&encoderRaw)+1)|
|DATA[6]|编码器零偏低字节|DATA[6] = *(uint8_t *)(&encoderOffset)|
|DATA[7]|编码器零偏高字节|DATA[7] = *((uint8_t *)(&encoderOffset)+1)|
### 24. 写入编码器值到ROM作为电机零点命令（1帧）

主机发送该命令以设置编码器的零偏，其中，需要写入的编码器值encoderOffset为uint16_t类型，14bit编码器的数值范围0~16383。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x91|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|编码器零偏低字节|DATA[6] = *(uint8_t *)(&encoderOffset)|
|DATA[7]|编码器零偏高字节|DATA[7] = *((uint8_t *)(&encoderOffset)+1)|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据和主机发送的命令相同。

### 25. 写入当前位置到ROM作为电机零点命令（1帧）

将电机当前编码器位置作为初始位置写入到ROM

#### 注意

1. 该命令需要重新上电后才能生效

2. 该命令会将零点写入驱动的ROM，多次写入将会影响芯片寿命，不建议频繁使用

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x19|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，数据中encoderOffset为设置的0偏值

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x19|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|编码器零偏低字节|DATA[6] = *(uint8_t *)(&encoderOffset)|
|DATA[7]|编码器零偏高字节|DATA[7] = *((uint8_t *)(&encoderOffset)+1)|
### 26. 读取多圈角度命令（1帧）

主机发送该命令以读取当前电机的多圈绝对角度值

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x92|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 电机角度motorAngle，为int64_t类型数据，正值表示顺时针累计角度，负值表示逆时针累计角度，单位0.01°/LSB。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x92|
|DATA[1]|角度低字节1|DATA[1] = *(uint8_t *)(&motorAngle)|
|DATA[2]|角度字节2|DATA[2] = *((uint8_t *)(&motorAngle)+1)|
|DATA[3]|角度字节3|DATA[3] = *((uint8_t *)(&motorAngle)+2)|
|DATA[4]|角度字节4|DATA[4] = *((uint8_t *)(&motorAngle)+3)|
|DATA[5]|角度字节5|DATA[5] = *((uint8_t *)(&motorAngle)+4)|
|DATA[6]|角度字节6|DATA[6] = *((uint8_t *)(&motorAngle)+5)|
|DATA[7]|角度字节7|DATA[7] = *((uint8_t *)(&motorAngle)+6)|
### 27. 读取单圈角度命令（1帧）

主机发送该命令以读取当前电机的单圈角度

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x94|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，该帧数据中包含了以下参数。

1. 电机单圈角度circleAngle，为uint32_t类型数据，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0，单位0.01°/LSB，数值范围0~36000*减速比-1。

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x94|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|单圈角度低字节1|DATA[4] = *(uint8_t *)(&circleAngle)|
|DATA[5]|单圈角度字节2|DATA[5] = *((uint8_t *)(&circleAngle)+1)|
|DATA[6]|单圈角度字节3|DATA[6] = *((uint8_t *)(&circleAngle)+2)|
|DATA[7]|单圈角度高字节4|DATA[7] = *((uint8_t *)(&circleAngle)+3)|
### 28. 清除电机角度命令（1帧）暂未实现

该命令清除电机的多圈和单圈角度数据，并将当前位置设为电机的零点，断电后失效

#### 注意

该命令会同时清除所有位置环的控制命令数据

|数据域|说明|数据|
|---|---|---|
|DATA[0]|命令字节|0x95|
|DATA[1]|NULL|0x00|
|DATA[2]|NULL|0x00|
|DATA[3]|NULL|0x00|
|DATA[4]|NULL|0x00|
|DATA[5]|NULL|0x00|
|DATA[6]|NULL|0x00|
|DATA[7]|NULL|0x00|
#### 驱动回复（1帧）

电机在收到命令后回复主机，帧数据和主机发送相同

---

### 使用说明

1. 复制上述全部内容，粘贴到Microsoft Word中

2. Word中可通过「开始」-「样式」调整标题层级格式

3. 表格可通过「表格工具」调整列宽、对齐方式等样式

4. 可根据需要添加页码、页眉页脚等文档元素

该文档完整保留了原始协议的所有内容，包括命令格式、数据域定义、参数说明、备注信息等，可直接用于工程开发和设备调试。
> （注：文档部分内容可能由 AI 生成）