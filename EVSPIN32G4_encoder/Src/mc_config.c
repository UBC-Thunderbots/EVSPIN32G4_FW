
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "pqd_motor_power_measurement.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .ConvFact = PQD_CONVERSION_FACTOR
};

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(IQMAX * SP_KIDIV),
  .wLowerIntegralLimit = -(int32_t)(IQMAX * SP_KIDIV),
  .hUpperOutputLimit   = (int16_t)IQMAX,
  .hLowerOutputLimit   = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1.
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1.
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1.
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque          = (int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque          = -(int16_t)NOMINAL_CURRENT,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT,
};

PWMC_R3_2_Handle_t PWM_Handle_M1 =
{
  ._Super =
  {
    .pFctGetPhaseCurrents       = &R3_2_GetPhaseCurrents,
    .pFctSetADCSampPointSectX   = &R3_2_SetADCSampPointSectX,
    .pFctSetOffsetCalib         = &R3_2_SetOffsetCalib,
    .pFctGetOffsetCalib         = &R3_2_GetOffsetCalib,
    .pFctSwitchOffPwm           = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm            = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib       = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides         = &R3_2_TurnOnLowSides,
    .pFctOCPSetReferenceVoltage = MC_NULL,
    .pFctRLDetectionModeEnable  = &R3_2_RLDetectionModeEnable,
    .pFctRLDetectionModeDisable = &R3_2_RLDetectionModeDisable,
    .pFctRLDetectionModeSetDuty = &R3_2_RLDetectionModeSetDuty,
    .pFctRLTurnOnLowSidesAndStart = &R3_2_RLTurnOnLowSidesAndStart,
    .hT_Sqrt3                   = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
    .pwm_en_u_port     = MC_NULL,
    .pwm_en_u_pin      = (uint16_t)0,
    .pwm_en_v_port     = MC_NULL,
    .pwm_en_v_pin      = (uint16_t)0,
    .pwm_en_w_port     = MC_NULL,
    .pwm_en_w_pin      = (uint16_t)0,
    .Sector                     = 0,
    .lowDuty                    = (uint16_t)0,
    .midDuty                    = (uint16_t)0,
    .highDuty                   = (uint16_t)0,
    .CntPhA                     = 0,
    .CntPhB                     = 0,
    .CntPhC                     = 0,
    .SWerror                    = 0,
    .TurnOnLowSidesAction       = false,
    .OffCalibrWaitTimeCounter   = 0,
    .Motor                      = M1,
    .RLDetectionMode            = false,
    .SingleShuntTopology        = false,
    .Ia                         = 0,
    .Ib                         = 0,
    .Ic                         = 0,
    .LPFIqd_const               = LPF_FILT_CONST,
    .DTCompCnt                  = DTCOMPCNT,
    .PWMperiod                  = PWM_PERIOD_CYCLES,
    .Ton                        = TON,
    .Toff                       = TOFF,
    .OverCurrentFlag            = false,
    .OverVoltageFlag            = false,
    .BrakeActionLock            = false,
    .driverProtectionFlag       = false,
  },

  .Half_PWMPeriod               = PWM_PERIOD_CYCLES/2u,
  .PhaseAOffset                 = 32767,
  .PhaseBOffset                 = 32767,
  .PhaseCOffset                 = 32767,
  .PolarizationCounter          = (uint8_t)0,
  .PolarizationSector           = (uint8_t)0,
  .pParams_str                  = &R3_2_ParamsM1
};

OpenLoop_Handle_t OpenLoop_ParamsM1 =
{
  .hDefaultVoltage = OPEN_LOOP_VOLTAGE_d,
  .VFMode          = OPEN_LOOP_VF,
  .hVFOffset       = OPEN_LOOP_OFF,
  .hVFSlope        = OPEN_LOOP_K
};

/** RAMP for Motor1
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component.
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule = MAX_MODULE,
  .MaxVd     = (uint16_t)((MAX_MODULE * 950) / 1000),
};

FOCVars_t FOCVars[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
PID_Handle_t *pPIDIq[NBR_OF_MOTORS]             = {&PIDIqHandle_M1};
PID_Handle_t *pPIDId[NBR_OF_MOTORS]             = {&PIDIdHandle_M1};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]  = {&PQD_MotorPowMeasM1};

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSTC = &SpeednTorqCtrlM1,
    .pFOCVars = &FOCVars[0],
    .pVSS = &VirtualSpeedSensorM1,
    .pPWM = &PWM_Handle_M1._Super,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

