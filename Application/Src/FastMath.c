#include "FastMath.h"
#include <stdint.h>

int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value) {
  uint32_t left = 0;
  uint32_t right = size - 1;
  uint32_t middle = (left + right) >> 1;
  while (right - left > 1) {
    if (source[middle] < value) {
      left = middle;
    }
    else if (source[middle] > value){
      right = middle;
    }
    else {
      return target[middle];
    }
    middle = (left + right) >> 1;
  }
  return (value - source[left]) * (target[right] - target[left]) / (source[right] - source[left]) + target[left];
}

float lookupTblf(const float* source, const float* target, const uint32_t size, const float value){
  uint32_t left = 0;
  uint32_t right = size - 1;
  uint32_t middle = (left + right) >> 1;
  while (right - left > 1) {
    if (source[middle] < value) {
      left = middle;
    }
    else if (source[middle] > value){
      right = middle;
    }
    else {
      return target[middle];
    }
    middle = (left + right) >> 1;
  }
  return (value - source[left]) * (target[right] - target[left]) / (source[right] - source[left]) + target[left];
}

float flookupTbll(const int32_t* source, const float* target, const uint32_t size, const int32_t value){
  uint32_t left = 0;
  uint32_t right = size - 1;
  uint32_t middle = (left + right) >> 1;
  while (right - left > 1) {
    if (source[middle] < value) {
      left = middle;
    }
    else if (source[middle] > value){
      right = middle;
    }
    else {
      return target[middle];
    }
    middle = (left + right) >> 1;
  }
  return (value - source[left]) * (target[right] - target[left]) / (source[right] - source[left]) + target[left];
}