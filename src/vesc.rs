use heapless::Vec;

pub enum VescCommand {
    SetDisp(DispPosMode),
    SetPos(f32),
    SetCurrent(f32),
    SetDuty(f32),
    SendPosRequest,
}

impl VescCommand {
    pub fn serialize(&self) -> Vec<u8, 32> {
        let payload = match self {
            VescCommand::SetDisp(mode) => {
                let mut data = Vec::<u8, 32>::new();
                data.push(COMM_PACKET_ID::COMM_SET_DETECT as u8).ok();
                data.push((*mode).into()).ok();
                data
            }
            VescCommand::SetPos(pos) => {
                let mut data = Vec::<u8, 32>::new();
                data.push(COMM_PACKET_ID::COMM_SET_POS as u8).ok();
                data.extend(((pos * 1000000.0) as i32).to_be_bytes().iter().cloned());
                data
            }
            VescCommand::SetCurrent(current) => {
                let mut data = Vec::<u8, 32>::new();
                data.push(COMM_PACKET_ID::COMM_SET_CURRENT as u8).ok();
                data.extend(((current * 1000.0) as i32).to_be_bytes().iter().cloned());
                data
            }
            VescCommand::SetDuty(duty) => {
                let mut data = Vec::<u8, 32>::new();
                data.push(COMM_PACKET_ID::COMM_SET_DUTY as u8).ok();
                data.extend(((duty * 100000.0) as i32).to_be_bytes().iter().cloned());
                data
            }
            VescCommand::SendPosRequest => {
                let mut data = Vec::<u8, 32>::new();
                data.push(COMM_PACKET_ID::COMM_GET_ROTOR_POSITION as u8)
                    .ok();
                data
            }
        };

        Self::create_vesc_command(payload)
    }

    fn create_vesc_command(payload: Vec<u8, 32>) -> Vec<u8, 32> {
        let crc = crc16(&payload);

        let mut data = Vec::<u8, 32>::new();

        let packet_length = payload.len();

        if packet_length <= 256 {
            data.push(2).ok();
            data.push(packet_length as u8).ok();
        } else {
            data.push(3).ok();
            data.push((packet_length >> 8) as u8).ok();
            data.push((packet_length & 0xFF) as u8).ok();
        }

        data.extend_from_slice(&payload).ok();

        data.push((crc >> 8) as u8).ok();
        data.push(crc as u8).ok();

        data.push(3).ok();

        data
    }
}

#[derive(Debug, defmt::Format)]
pub enum VescReply {
    MotorPosition(f32),
}

impl VescReply {
    pub fn parse_from_buffer(buffer: &[u8; 10]) -> Option<Self> {
        // il pacchetto del rotore essere tipo
        // 2 5 COMM_ROTOR_POSITION x x x x crc crc 3

        if buffer[0] != 2 {
            return None;
        }
        if buffer[1] != 5 {
            return None;
        }
        if buffer[2] != COMM_PACKET_ID::COMM_ROTOR_POSITION as u8 {
            return None;
        }
        /*if bytes == 10 {
            //println!("Received: {:?}", buffer);
        }*/

        let rotor_position = (buffer[3] as i32) << 24
            | (buffer[4] as i32) << 16
            | (buffer[5] as i32) << 8
            | buffer[6] as i32;

        Some(Self::MotorPosition(rotor_position as f32 / 100000.0))
    }
}

fn crc16(buf: &Vec<u8, 32>) -> u16 {
    let crc16_tab: &'static [u16] = &[
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a,
        0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294,
        0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462,
        0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509,
        0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695,
        0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
        0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
        0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
        0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b,
        0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f,
        0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
        0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046,
        0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290,
        0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e,
        0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691,
        0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
        0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d,
        0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,
        0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8,
        0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e,
        0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93,
        0x3eb2, 0x0ed1, 0x1ef0,
    ];

    let mut cksum: u16 = 0;
    for i in 0..buf.len() {
        cksum = crc16_tab[(((cksum >> 8 as u16) ^ buf[i as usize] as u16) & 0xFF) as usize]
            ^ (cksum << 8);
    }
    cksum
}

#[repr(u8)]
enum COMM_PACKET_ID {
    COMM_FW_VERSION,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_HANDBRAKE,
    COMM_SET_DETECT,
    COMM_SET_SERVO_POS,
    COMM_SET_MCCONF,
    COMM_GET_MCCONF,
    COMM_GET_MCCONF_DEFAULT,
    COMM_SET_APPCONF,
    COMM_GET_APPCONF,
    COMM_GET_APPCONF_DEFAULT,
    COMM_SAMPLE_PRINT,
    COMM_TERMINAL_CMD,
    COMM_PRINT,
    COMM_ROTOR_POSITION,
    COMM_EXPERIMENT_SAMPLE,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_MOTOR_R_L,
    COMM_DETECT_MOTOR_FLUX_LINKAGE,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL_FOC,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_DECODED_PPM,
    COMM_GET_DECODED_ADC,
    COMM_GET_DECODED_CHUK,
    COMM_FORWARD_CAN,
    COMM_SET_CHUCK_DATA,
    COMM_CUSTOM_APP_DATA,
    COMM_NRF_START_PAIRING,
    COMM_GPD_SET_FSW,
    COMM_GPD_BUFFER_NOTIFY,
    COMM_GPD_BUFFER_SIZE_LEFT,
    COMM_GPD_FILL_BUFFER,
    COMM_GPD_OUTPUT_SAMPLE,
    COMM_GPD_SET_MODE,
    COMM_GPD_FILL_BUFFER_INT8,
    COMM_GPD_FILL_BUFFER_INT16,
    COMM_GPD_SET_BUFFER_INT_SCALE,
    COMM_GET_VALUES_SETUP,
    COMM_SET_MCCONF_TEMP,
    COMM_SET_MCCONF_TEMP_SETUP,
    COMM_GET_VALUES_SELECTIVE,
    COMM_GET_VALUES_SETUP_SELECTIVE,
    COMM_EXT_NRF_PRESENT,
    COMM_EXT_NRF_ESB_SET_CH_ADDR,
    COMM_EXT_NRF_ESB_SEND_DATA,
    COMM_EXT_NRF_ESB_RX_DATA,
    COMM_EXT_NRF_SET_ENABLED,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
    COMM_DETECT_APPLY_ALL_FOC,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
    COMM_ERASE_NEW_APP_ALL_CAN,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN,
    COMM_PING_CAN,
    COMM_APP_DISABLE_OUTPUT,
    COMM_TERMINAL_CMD_SYNC,
    COMM_GET_IMU_DATA,
    COMM_BM_CONNECT,
    COMM_BM_ERASE_FLASH_ALL,
    COMM_BM_WRITE_FLASH,
    COMM_BM_REBOOT,
    COMM_BM_DISCONNECT,
    COMM_BM_MAP_PINS_DEFAULT,
    COMM_BM_MAP_PINS_NRF5X,
    COMM_ERASE_BOOTLOADER,
    COMM_ERASE_BOOTLOADER_ALL_CAN,
    COMM_PLOT_INIT,
    COMM_PLOT_DATA,
    COMM_PLOT_ADD_GRAPH,
    COMM_PLOT_SET_GRAPH,
    COMM_GET_DECODED_BALANCE,
    COMM_BM_MEM_READ,
    COMM_WRITE_NEW_APP_DATA_LZO,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
    COMM_BM_WRITE_FLASH_LZO,
    COMM_SET_CURRENT_REL,
    COMM_CAN_FWD_FRAME,
    COMM_SET_BATTERY_CUT,
    COMM_SET_BLE_NAME,
    COMM_SET_BLE_PIN,
    COMM_SET_CAN_MODE,
    COMM_GET_IMU_CALIBRATION,
    COMM_GET_ROTOR_POSITION,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum DispPosMode {
    DISP_POS_MODE_NONE,
    DISP_POS_MODE_INDUCTANCE,
    DISP_POS_MODE_OBSERVER,
    DISP_POS_MODE_ENCODER,
    DISP_POS_MODE_PID_POS,
    DISP_POS_MODE_PID_POS_ERROR,
    DISP_POS_MODE_ENCODER_OBSERVER_ERROR,
}

impl From<DispPosMode> for u8 {
    fn from(mode: DispPosMode) -> Self {
        match mode {
            DispPosMode::DISP_POS_MODE_NONE => 0,
            DispPosMode::DISP_POS_MODE_INDUCTANCE => 1,
            DispPosMode::DISP_POS_MODE_OBSERVER => 2,
            DispPosMode::DISP_POS_MODE_ENCODER => 3,
            DispPosMode::DISP_POS_MODE_PID_POS => 4,
            DispPosMode::DISP_POS_MODE_PID_POS_ERROR => 5,
            DispPosMode::DISP_POS_MODE_ENCODER_OBSERVER_ERROR => 6,
        }
    }
}
