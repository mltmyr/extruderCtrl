#ifndef THERMISTOR_104GT_AND_RAMPS14_TABLE_H__
#define THERMISTOR_104GT_AND_RAMPS14_TABLE_H__

#include "TableThermistor.h"

const uint8_t             therm_104gt_ramps14_table_len = 32;
const temperature_entry_t therm_104gt_ramps14_table[] PROGMEM = {
    {   1, 714},
    {  22, 300},
    {  25, 290},
    {  29, 280},
    {  33, 270},
    {  38, 260},
    {  44, 250},
    {  51, 240},
    {  60, 230},
    {  71, 220},
    {  83, 210},
    {  99, 200},
    { 118, 190},
    { 140, 180},
    { 168, 170},
    { 202, 160},
    { 242, 150},
    { 290, 140},
    { 346, 130},
    { 410, 120},
    { 482, 110},
    { 558, 100},
    { 636,  90},
    { 712,  80},
    { 783,  70},
    { 844,  60},
    { 896,  50},
    { 936,  40},
    { 966,  30},
    { 987,  20},
    {1001,  10},
    {1010,   0}
};

#endif /* THERMISTOR_104GT_AND_RAMPS14_TABLE_H__ */