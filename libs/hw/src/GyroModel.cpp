#include "GyroModel.h"

void hw::GyroModel::SetAngularVelocityRadps(double w_radps) { this->w_radps = w_radps; }
double hw::GyroModel::GetAngularVelocityRadps() { return w_radps; }
void hw::GyroModel::Clear() { w_radps = 0; }