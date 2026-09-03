// 原始点云/图像 C ABI：只校验消息映射和大缓冲布局，不依赖或调用定位 Engine。
// 已发布函数仍接收主 handle 以保持 ABI；真正感知算法由独立 ROS 2 前端负责。
#include "zju_coop/c_api.h"

#include "api/c_api_detail.hpp"

#include <cstdint>

namespace {

using zju::coop::c_api_detail::all_zero;
using zju::coop::c_api_detail::array_element;
using zju::coop::c_api_detail::nul_terminated_nonempty;
using zju::coop::c_api_detail::valid_array_span;
using zju::coop::c_api_detail::valid_boolean;
using zju::coop::c_api_detail::validate_header;

std::uint32_t point_field_element_size(
    zju_coop_point_field_datatype_t datatype) {
  switch (datatype) {
    case ZJU_COOP_POINT_FIELD_INT8:
    case ZJU_COOP_POINT_FIELD_UINT8:
      return 1U;
    case ZJU_COOP_POINT_FIELD_INT16:
    case ZJU_COOP_POINT_FIELD_UINT16:
      return 2U;
    case ZJU_COOP_POINT_FIELD_INT32:
    case ZJU_COOP_POINT_FIELD_UINT32:
    case ZJU_COOP_POINT_FIELD_FLOAT32:
      return 4U;
    case ZJU_COOP_POINT_FIELD_FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

zju_coop_error_code_t validate_point_cloud_packet(
    const zju_coop_point_cloud_packet_t& packet) {
  if (packet.node_id == 0U || packet.sensor_id == 0U ||
      packet.sequence == 0U || packet.timestamp_ns == 0U ||
      packet.receive_timestamp_ns == 0U ||
      !nul_terminated_nonempty(packet.frame_id, sizeof(packet.frame_id)) ||
      packet.height == 0U || packet.width == 0U ||
      packet.field_count == 0U || packet.point_step == 0U ||
      packet.row_step == 0U || packet.data == nullptr ||
      !valid_boolean(packet.is_bigendian) ||
      !valid_boolean(packet.is_dense) || !valid_boolean(packet.valid) ||
      packet.status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !all_zero(packet.reserved0)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  const std::uint64_t minimum_row_step =
      static_cast<std::uint64_t>(packet.point_step) * packet.width;
  const std::uint64_t expected_data_size =
      static_cast<std::uint64_t>(packet.row_step) * packet.height;
  if (packet.row_step < minimum_row_step ||
      packet.data_size != expected_data_size ||
      !valid_array_span<zju_coop_point_field_t>(
          packet.fields, packet.field_count, packet.field_stride)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  for (std::uint32_t index = 0U; index < packet.field_count; ++index) {
    const auto* field = array_element<zju_coop_point_field_t>(
        packet.fields, index, packet.field_stride);
    const auto header_status = validate_header(field);
    if (header_status != ZJU_COOP_OK) {
      return header_status;
    }
    const std::uint32_t element_size =
        point_field_element_size(field->datatype);
    const std::uint64_t field_end =
        static_cast<std::uint64_t>(field->offset) +
        static_cast<std::uint64_t>(element_size) * field->count;
    if (field->struct_size > packet.field_stride ||
        !nul_terminated_nonempty(field->name, sizeof(field->name)) ||
        element_size == 0U || field->count == 0U ||
        !all_zero(field->reserved0) || field->reserved1 != 0U ||
        field_end > packet.point_step) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
  }
  return ZJU_COOP_OK;
}

zju_coop_error_code_t validate_camera_image_packet(
    const zju_coop_camera_image_packet_t& packet) {
  const std::uint64_t expected_data_size =
      static_cast<std::uint64_t>(packet.step) * packet.height;
  if (packet.node_id == 0U || packet.camera_id == 0U ||
      packet.sequence == 0U || packet.timestamp_ns == 0U ||
      packet.receive_timestamp_ns == 0U ||
      !nul_terminated_nonempty(packet.frame_id, sizeof(packet.frame_id)) ||
      !nul_terminated_nonempty(packet.encoding, sizeof(packet.encoding)) ||
      packet.height == 0U || packet.width == 0U ||
      packet.step < packet.width || packet.reserved0 != 0U ||
      packet.data == nullptr || packet.data_size != expected_data_size ||
      !valid_boolean(packet.is_bigendian) || !valid_boolean(packet.valid) ||
      packet.status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !all_zero(packet.reserved1)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  return ZJU_COOP_OK;
}

void fill_raw_input_result(zju_coop_raw_input_result_t& destination,
                           zju_coop_raw_input_type_t input_type,
                           std::uint32_t node_id, std::uint32_t sensor_id,
                           std::uint64_t sequence,
                           std::uint64_t timestamp_ns) {
  zju_coop_raw_input_result_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;
  output.input_type = input_type;
  output.disposition = ZJU_COOP_RAW_INPUT_VALIDATED_NOT_USED;
  output.node_id = node_id;
  output.sensor_id = sensor_id;
  output.sequence = sequence;
  output.timestamp_ns = timestamp_ns;
  const std::uint32_t caller_size = destination.struct_size;
  destination = output;
  destination.struct_size = caller_size;
}

}  // namespace

extern "C" {

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_point_field_init(
    zju_coop_point_field_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_point_cloud_packet_init(
    zju_coop_point_cloud_packet_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->field_stride = sizeof(zju_coop_point_field_t);
  value->status = ZJU_COOP_RANGE_STATUS_OK;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_camera_image_packet_init(
    zju_coop_camera_image_packet_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->status = ZJU_COOP_RANGE_STATUS_OK;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_raw_input_result_init(
    zju_coop_raw_input_result_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->input_type = ZJU_COOP_RAW_INPUT_UNKNOWN;
  value->disposition = ZJU_COOP_RAW_INPUT_UNINITIALIZED;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_point_cloud(
    zju_coop_handle_t* handle,
    const zju_coop_point_cloud_packet_t* packet,
    zju_coop_raw_input_result_t* result) {
  // handle 只作为既有会话有效性令牌；raw 输入不会访问 Engine 状态。
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto packet_header_status = validate_header(packet);
  if (packet_header_status != ZJU_COOP_OK) {
    return packet_header_status;
  }
  const auto result_header_status = validate_header(result);
  if (result_header_status != ZJU_COOP_OK) {
    return result_header_status;
  }
  const auto packet_status = validate_point_cloud_packet(*packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;
  }

  fill_raw_input_result(*result, ZJU_COOP_RAW_INPUT_POINT_CLOUD,
                        packet->node_id, packet->sensor_id,
                        packet->sequence, packet->timestamp_ns);
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_camera_image(
    zju_coop_handle_t* handle,
    const zju_coop_camera_image_packet_t* packet,
    zju_coop_raw_input_result_t* result) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto packet_header_status = validate_header(packet);
  if (packet_header_status != ZJU_COOP_OK) {
    return packet_header_status;
  }
  const auto result_header_status = validate_header(result);
  if (result_header_status != ZJU_COOP_OK) {
    return result_header_status;
  }
  const auto packet_status = validate_camera_image_packet(*packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;
  }

  fill_raw_input_result(*result, ZJU_COOP_RAW_INPUT_CAMERA_IMAGE,
                        packet->node_id, packet->camera_id,
                        packet->sequence, packet->timestamp_ns);
  return ZJU_COOP_OK;
}

}  // extern "C"
