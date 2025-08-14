//#include "mixer_out.h"
//
///**
// * @brief 对输入值进行范围限制，确保其处于指定的最小值和最大值之间。
// *
// * 该函数会检查输入值 `val` 是否小于指定的最小值 `min` 或者大于指定的最大值 `max`。
// * 如果 `val` 小于 `min`，则返回 `min`；如果 `val` 大于 `max`，则返回 `max`；
// * 如果 `val` 处于 `min` 和 `max` 之间，则返回 `val` 本身。
// *
// * @param val 待限制范围的输入值。
// * @param min 允许的最小值。
// * @param max 允许的最大值。
// * @return float 经过范围限制后的值，该值一定处于 [min, max] 区间内。
// */
//static inline float constrain(float val, float min, float max)
//{
//    return val < min ? min : val > max ? max : val;
//}
//
//
///**
// * @brief 计算用于减少输出饱和度的增益值。
// *
// * 该函数会遍历所有电机的输出值，根据每个输出值是否超出允许的最小或最大值，计算相应的增益值。
// * 最终将这些增益值综合起来，得到一个可以用于调整输出以减少饱和度的总增益值。
// *
// * @param desaturation_vector 去饱和向量，用于计算每个电机输出的调整量。
// * @param outputs 电机的输出值数组，存储每个电机当前的输出。
// * @param sat_status 饱和状态结构体指针，用于记录电机输出是否在正向或负向饱和。
// * @param min_output 允许的最小输出值，输出值不能低于此值。
// * @param max_output 允许的最大输出值，输出值不能高于此值。
// * @return float 计算得到的去饱和增益值，用于后续调整输出以减少饱和度。
// */
//float compute_desaturation_gain(float *desaturation_vector, float *outputs, saturation_status_t *sat_status,
//                                float min_output, float max_output)
//{
//    // 初始化最小增益值为 0
//    float k_min = 0.0f;
//    // 初始化最大增益值为 0
//    float k_max = 0.0f;
//
//    // 遍历所有电机的输出
//    for (int i = 0; i < _rotor_count; i++) {
//        // 避免除零错误。如果 desaturation_vector[i] 为零，则无法进行不饱和处理
//        if (fabsf(desaturation_vector[i]) < FLT_EPSILON) {
//            // 若去饱和向量当前元素接近零，跳过此次循环，避免除零错误
//            continue;
//        }
//
//        // 检查当前电机输出是否低于最小允许输出
//        if (outputs[i] < min_output) {
//            // 计算需要调整的增益值，使输出达到最小允许值
//            float k = (min_output - outputs[i]) / desaturation_vector[i];
//
//            // 更新最小增益值
//            if (k < k_min) {
//                k_min = k;
//            }
//
//            // 更新最大增益值
//            if (k > k_max) {
//                k_max = k;
//            }
//
//            // 标记电机输出在负向饱和
//            sat_status->flags.motor_neg = true;
//        }
//
//        // 检查当前电机输出是否高于最大允许输出
//        if (outputs[i] > max_output) {
//            // 计算需要调整的增益值，使输出达到最大允许值
//            float k = (max_output - outputs[i]) / desaturation_vector[i];
//
//            // 更新最小增益值
//            if (k < k_min) {
//                k_min = k;
//            }
//
//            // 更新最大增益值
//            if (k > k_max) {
//                k_max = k;
//            }
//
//            // 标记电机输出在正向饱和
//            sat_status->flags.motor_pos = true;
//        }
//    }
//
//    // 尽可能减少饱和度，返回最小和最大增益值之和
//    return k_min + k_max;
//}
//
//
///**
// * @brief 最小化输出的饱和度，通过计算并应用去饱和增益来调整输出值。
// *
// * 该函数通过两次计算去饱和增益并将其应用到输出值上，以尽可能减少输出的饱和度。
// * 第一次计算增益并应用后，会再次计算增益，在某些情况下，第二次计算的增益不为零，
// * 此时将其一半应用到输出值上，有助于平衡饱和度。
// *
// * @param desaturation_vector 去饱和向量，用于计算每个电机输出的调整量。
// * @param outputs 电机的输出值数组，存储每个电机当前的输出。
// * @param sat_status 饱和状态结构体指针，用于记录电机输出是否在正向或负向饱和。
// * @param min_output 允许的最小输出值，输出值不能低于此值。
// * @param max_output 允许的最大输出值，输出值不能高于此值。
// */
//void minimize_saturation(float *desaturation_vector, float *outputs, saturation_status_t *sat_status, float min_output,
//                         float max_output)
//{
//    // 第一次计算去饱和增益
//    float k1 = compute_desaturation_gain(desaturation_vector, outputs, sat_status, min_output, max_output);
//
//    // 将第一次计算得到的增益应用到每个电机的输出上
//    for (int i = 0; i < _rotor_count; i++) {
//        outputs[i] += k1 * desaturation_vector[i];
//    }
//
//    // 在更新后的输出上再次计算去饱和增益。
//    // 在大多数情况下，它将为零。如果 max(outputs) - min(outputs) > max_output - min_output，则不为零。
//    // 在这种情况下，添加 0.5 倍的增益将平衡饱和度
//    float k2 = 0.5f * compute_desaturation_gain(desaturation_vector, outputs, sat_status, min_output, max_output);
//
//    // 将第二次计算得到的增益的一半应用到每个电机的输出上
//    for (int i = 0; i < _rotor_count; i++) {
//        outputs[i] += k2 * desaturation_vector[i];
//    }
//}
//
///**
// * @brief 更新控制饱和状态报告。
// *
// * 该函数根据指定电机的输出是否达到上限或下限饱和，分析并记录是哪些控制轴（滚动、俯仰、偏航、推力）
// * 的正向或负向变化导致了饱和。最终将饱和状态报告标记为有效。
// *
// * @param index 标识正在饱和的电机的基于 0 的索引。
// * @param clipping_high 如果电机需求在正向被限制（达到上限饱和），则为 true。
// * @param clipping_low 如果电机需求在负向被限制（达到下限饱和），则为 true。
// */
//static void update_saturation_status(int index, bool clipping_high, bool clipping_low)
//{
//    // 电机在上限饱和
//    // 检查哪些控制轴和方向正在导致饱和
//    if (clipping_high) {
//        // 检查滚动输入是否饱和
//        if (_rotors[index].roll_scale > 0.0f) {
//            // 当 _rotors[index].roll_scale 为正，滚动正向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.roll_pos = true;
//
//        } else if (_rotors[index].roll_scale < 0.0f) {
//            // 当 _rotors[index].roll_scale 为负，滚动负向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.roll_neg = true;
//        }
//
//        // 检查俯仰输入是否饱和
//        if (_rotors[index].pitch_scale > 0.0f) {
//            // 当 _rotors[index].pitch_scale 为正，俯仰正向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.pitch_pos = true;
//
//        } else if (_rotors[index].pitch_scale < 0.0f) {
//            // 当 _rotors[index].pitch_scale 为负，俯仰负向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.pitch_neg = true;
//        }
//
//        // 检查偏航输入是否饱和
//        if (_rotors[index].yaw_scale > 0.0f) {
//            // 当 _rotors[index].yaw_scale 为正，偏航正向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.yaw_pos = true;
//
//        } else if (_rotors[index].yaw_scale < 0.0f) {
//            // 当 _rotors[index].yaw_scale 为负，偏航负向变化会使电机输出更接近上限，从而增加饱和度
//            _saturation_status.flags.yaw_neg = true;
//        }
//
//        // 推力正向变化会使电机输出更接近上限，从而增加饱和度
//        _saturation_status.flags.thrust_pos = true;
//    }
//
//    // 电机在下限饱和
//    // 检查哪些控制轴和方向正在导致饱和
//    if (clipping_low) {
//        // 检查滚动输入是否饱和
//        if (_rotors[index].roll_scale > 0.0f) {
//            // 当 _rotors[index].roll_scale 为正，滚动负向变化会使电机输出更接近下限，从而增加饱和度
//            _saturation_status.flags.roll_neg = true;
//
//        } else if (_rotors[index].roll_scale < 0.0f) {
//            // 当 _rotors[index].roll_scale 为负，滚动正向变化会使电机输出更接近下限，从而增加饱和度
//            _saturation_status.flags.roll_pos = true;
//        }
//
//        // 检查俯仰输入是否饱和
//        if (_rotors[index].pitch_scale > 0.0f) {
//            // 当 _rotors[index].pitch_scale 为正，俯仰负向变化会使电机输出更接近下限，从而增加饱和度
//            _saturation_status.flags.pitch_neg = true;
//
//        } else if (_rotors[index].pitch_scale < 0.0f) {
//            // 当 _rotors[index].pitch_scale 为负，俯仰正向变化会使电机输出更接近下限，从而增加饱和度
//            _saturation_status.flags.pitch_pos = true;
//        }
//
//        // 检查偏航输入是否饱和
//        if (_rotors[index].yaw_scale > 0.0f) {
//            // 当 _rotors[index].yaw_scale 为正，偏航负向变化会使电机输出更接近下限，从而增加饱和度
//            _saturation_status.flags.yaw_neg = true;
//
//        } else if (_rotors[index].yaw_scale < 0.0f) {
//            // 当 _rotors[index].yaw_scale 为负，偏航正向变化会使电机输出远离下限，即减少饱和度
//            _saturation_status.flags.yaw_pos = true;
//        }
//
//        // 推力负向变化会使电机输出更接近下限，从而增加饱和度
//        _saturation_status.flags.thrust_neg = true;
//    }
//
//    // 标记饱和状态报告为有效
//    _saturation_status.flags.valid = true;
//}
//
//
//
//
//
//
//
//
//
//
