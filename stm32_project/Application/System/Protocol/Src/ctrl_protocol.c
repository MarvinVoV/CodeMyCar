/*
 * ctrl_protocol.c
 *
 *  Created on: Feb 6, 2025
 *      Author: marvin
 */
#include "ctrl_protocol.h"

#include <stdbool.h>

/**
 * @brief 检查是否包含指定控制字段
 * @param fields 当前控制字段掩码
 * @param field 要检查的字段掩码
 * @return 包含返回true，否则false
 */
bool isCtrlFieldSet(const uint8_t fields, const CtrlField field)
{
    return (fields & field) == field;
}

bool isCtrlFieldsSet(const uint8_t fields, const uint8_t checkFields)
{
    return (fields & checkFields) == checkFields;
}


bool validateControlCmd(const uint8_t* cmdPayload, uint16_t totalLen, ControlCmdError* error)
{
    // todo
    return true;
}
