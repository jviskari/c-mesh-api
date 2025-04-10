/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#ifndef PROTO_CONFIG_H_
#define PROTO_CONFIG_H_

#include "generic_message.pb.h"
#include "wpc.h"
#include "wpc_proto.h"

/**
 * \brief   Force a refresh of the otap variables
 */
void Proto_config_refresh_otap_infos();

/**
 * \brief   Intialize the module in charge of data handling config
 * \return  True if successful, False otherwise
 */
bool Proto_config_init(void);

/**
 * \brief   Close the module in charge of config
 */
void Proto_config_close();

/**
 * \brief   Handle Get scratchpad status
 * \param   req
 *          Pointer to the request received
 * \param   resp
 *          Pointer to the reponse to send back
 * \return  APP_RES_PROTO_OK if answer is ready to send
 */
app_proto_res_e Proto_config_handle_get_scratchpad_status(wp_GetScratchpadStatusReq * req,
                                                          wp_GetScratchpadStatusResp * resp);

/**
 * \brief   Handle Set config request
 *          Apply new configs if any param has changed.
 *          Try to apply all of them, even if one fails.
 * \param   req
 *          Pointer to the request received
 * \param   resp
 *          Pointer to the reponse to send back
 * \return  APP_RES_PROTO_OK if answer is ready to send
 */
app_proto_res_e Proto_config_handle_set_config(wp_SetConfigReq *req,
                                               wp_SetConfigResp *resp);

/**
 * \brief   Handle Get config request
 * \param   req
 *          Pointer to the request received
 * \param   resp
 *          Pointer to the reponse to send back
 * \return  APP_RES_PROTO_OK if answer is ready to send
 */
app_proto_res_e Proto_config_handle_get_configs(wp_GetConfigsReq *req,
                                                wp_GetConfigsResp *resp);

/**
 * \brief   Handle Get gateway info
 * \param   req
 *          Pointer to the request received
 * \param   resp
 *          Pointer to the reponse to send back
 * \return  APP_RES_PROTO_OK if answer is ready to send
 */
app_proto_res_e Proto_config_handle_get_gateway_info_request(wp_GetGwInfoReq * req,
                                                             wp_GetGwInfoResp * resp);

/**
 * \brief   Handle Set scratchpad target and action
 * \param   req
 *          Pointer to the request received
 * \param   resp
 *          Pointer to the reponse to send back
 * \return  APP_RES_PROTO_OK if answer is ready to send
 */
app_proto_res_e Proto_config_handle_set_scratchpad_target_and_action_request(
                                            wp_SetScratchpadTargetAndActionReq *req,
                                            wp_SetScratchpadTargetAndActionResp *resp);

app_proto_res_e Proto_config_get_current_event_status(bool gw_online,
                                                      bool sink_online,
                                                      uint8_t * event_status_p,
                                                      size_t * event_status_size_p);

app_proto_res_e Proto_config_register_for_event_status(onEventStatus_cb_f onProtoEventStatus_cb);

/**
 * \brief   Get current sink address
 */
net_addr_t Proto_config_get_network_address(void);

#endif
