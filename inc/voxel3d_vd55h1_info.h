/**
 @file      voxel3d_vd55h1_info.h
 @brief     Header of voxel3d_vd55h1_info.cpp
 @author    Jackie Lee
 @copyright Copyright (c) 2025 5Voxel Co., Ltd.
 */

#ifndef __VOXEL3D_VD55H1_INFO_H__
#define __VOXEL3D_VD55H1_INFO_H__

#include <string>

#define VD55H1_REG_ADDR_SYSTEM_FSM			(0x54)
#define VD55H1_REG_ADDR_BOOT_FSM			(0x55)
#define VD55H1_REG_ADDR_ERROR_CODE			(0xB0)
#define VD55H1_REG_ADDR_WARNING_CODE		(0xB2)

std::string get_name_by_err_code(unsigned short error_code);
std::string get_name_by_system_fsm_code(unsigned short sys_fsm_code);
std::string get_name_by_boot_fsm_code(unsigned short boot_fsm_code);
std::string get_name_by_reg_addr(unsigned short reg_addr);

#endif /* __VOXEL3D_VD55H1_INFO_H__ */
