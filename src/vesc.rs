#![allow(non_camel_case_types, non_camel_case_variants)]

use defmt::warn;
use embedded_io_async::Read;
use heapless::Vec;

pub enum VescCommand {
    SetDisp(DispPosMode),
    SetPos(f32),
    SetCurrent(f32),
    SetDuty(f32),
    SendPosRequest,
    GenericCommand(COMM_PACKET_ID),
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
            VescCommand::GenericCommand(command) => {
                let mut data = Vec::<u8, 32>::new();
                data.push((*command).as_u8()).ok();
                data
            }
        };

        Self::create_vesc_command(&payload)
    }

    fn create_vesc_command(payload: &[u8]) -> Vec<u8, 32> {
        let mut digest = VESC_CRC.digest();
        digest.update(payload);
        let crc = digest.finalize();

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

        data.extend_from_slice(payload).ok();

        data.push((crc >> 8) as u8).ok();
        data.push(crc as u8).ok();

        data.push(3).ok();

        data
    }
}

#[derive(Debug, defmt::Format)]
pub enum VescReply {
    MotorPosition(f32),
    FirmwareVersion(u32),
    Unknown(Vec<u8, 32>),
}

impl VescReply {
    // il pacchetto del rotore essere tipo
    // 2 SIZE [PAYLOAD...] crc crc 3

    pub async fn read_vesc_packet(rx: &mut impl Read) -> Option<Vec<u8, 32>> {
        let mut byte = [0u8; 1];

        // Sync to start marker (type 2). We don't care about type 3 packets for now.
        while byte[0] != 2 {
            rx.read_exact(&mut byte).await.ok()?;
        }

        rx.read_exact(&mut byte).await.ok()?;
        let size = byte[0] as usize;
        if size > 32 {
            warn!("VESC packet too large: {} bytes", size);
            return None;
        }

        let mut payload_buf = [0u8; 32];
        rx.read_exact(&mut payload_buf[..size]).await.ok()?;

        let mut payload = heapless::Vec::<u8, 32>::new();
        payload.extend_from_slice(&payload_buf[..size]).ok()?;

        let mut crc = [0u8; 2];
        rx.read_exact(&mut crc).await.ok()?;

        let mut digest = VESC_CRC.digest();
        digest.update(&payload);
        let computed_crc = digest.finalize();

        let got_crc = u16::from_be_bytes(crc);
        if computed_crc != got_crc {
            warn!(
                "VESC CRC mismatch: computed={:04x} got={:04x}",
                computed_crc, got_crc
            );
            return None;
        }

        rx.read_exact(&mut byte).await.ok()?;
        if byte[0] != 3 {
            warn!("VESC packet end marker mismatch: got={}", byte[0]);
            return None;
        }

        Some(payload)
    }

    pub async fn eat_vesc_packet(rx: &mut impl Read) -> Option<Self> {
        // il pacchetto del rotore essere tipo
        // 2 5 COMM_ROTOR_POSITION x x x x crc crc 3

        // quindi ci aspettiamo ora un [COMM_ROTOR_POSITION x x x x]

        let payload = Self::read_vesc_packet(rx).await?;

        match payload.as_slice() {
            [COMM_ROTOR_POSITION_U8, pos_bytes @ ..] => {
                let rotor_position = u32::from_be_bytes(pos_bytes.try_into().ok()?);

                Some(Self::MotorPosition(rotor_position as f32 / 100000.0))
            }
            _ => {
                warn!("VESC packet parse failed: {:?}", payload);
                Some(Self::Unknown(payload))
            }
        }
    }
}

const VESC_CRC_ALG: crc::Algorithm<u16> = crc::Algorithm {
    width: 16,
    poly: 0x1021,
    init: 0x0000,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0x0000,
    residue: 0x0000,
};

const VESC_CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&VESC_CRC_ALG);

#[repr(u8)]
#[allow(non_camel_case_types)]
#[allow(non_camel_case_variants)]
#[derive(Copy, Clone)]
pub enum COMM_PACKET_ID {
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

impl COMM_PACKET_ID {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

const COMM_ROTOR_POSITION_U8: u8 = COMM_PACKET_ID::COMM_ROTOR_POSITION.as_u8();

#[repr(u8)]
#[allow(non_camel_case_variants)]
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
