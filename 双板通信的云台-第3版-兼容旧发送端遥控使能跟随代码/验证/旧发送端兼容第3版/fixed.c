
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>
#define USB_YAW_DEBUG 0
#define USB_ATS_COMPAT_FEEDBACK_TX 1
#define USB_LEGACY_VISION_TX 1
typedef struct { int mode; } Command;
static Command ReceivedPacketVision, vision_cmd;
static struct { float vx,vy,wz; } Navigation_speed_set;
static uint32_t nav_last_rx, vision_last_rx, tick, rx_advance;
static uint8_t nav_override, usb_nav_compat_active;
static uint8_t usb_nav_compat_allowed(uint32_t now) { (void)now; return 0; }
static int inject_vision, inject_nav, resolved, failures;
static jmp_buf stop_loop;
static uint32_t HAL_GetTick(void) { return tick; }
static void MX_USB_DEVICE_Init(void) {}
static void Update_to_resolve(void) { ++resolved; }
static void CAN_CmdBoard3(void *p) { (void)p; }
static void unified_usb_send_ats_feedback(void) {}
static void osDelay(int delay) { (void)delay; longjmp(stop_loop,1); }
static void unified_usb_process(void) {
    tick += rx_advance;
    if (inject_vision) {
        vision_last_rx=HAL_GetTick();
        vision_cmd.mode=1;
        if (!nav_override) ReceivedPacketVision=vision_cmd;
    }
    if (inject_nav) {
        nav_last_rx=HAL_GetTick();
        nav_override=1;
        ReceivedPacketVision.mode=2;
        Navigation_speed_set.vx=3;
    }
}
void usb_task(void const *argument)
{
  uint32_t last_state = 0;
#if !USB_YAW_DEBUG
  uint32_t last_chassis = 0;
#endif
  MX_USB_DEVICE_Init();
  while (1)
  {
    uint32_t now = HAL_GetTick();
#if USB_YAW_DEBUG
    if ((uint32_t)(now - last_state) >= 10U) {
      last_state = now;
      usb_send_yaw_debug();
    }
#else
    unified_usb_process();
    
    now = HAL_GetTick();
    
    if (usb_nav_compat_active && !usb_nav_compat_allowed(now)) {
      usb_nav_compat_active = 0U;
      nav_override = 0U;
      if ((uint32_t)(now - vision_last_rx) <= 100U) {
        ReceivedPacketVision = vision_cmd;
      } else {
        ReceivedPacketVision.mode = 0;
      }
      Update_to_resolve();
    }
    if ((uint32_t)(now - last_chassis) >= 10U) {
      last_chassis = now;
      if ((uint32_t)(now - nav_last_rx) > 100U) {
        Navigation_speed_set.vx = 0;
        Navigation_speed_set.vy = 0;
        Navigation_speed_set.wz = 0;
        if (nav_override) {
          usb_nav_compat_active = 0U;
          nav_override = 0;
          if ((uint32_t)(now - vision_last_rx) <= 100U) {
            ReceivedPacketVision = vision_cmd;
          } else {
            ReceivedPacketVision.mode = 0;
          }
          Update_to_resolve();
        }
      }
      CAN_CmdBoard3(&Navigation_speed_set);
    }
    if ((uint32_t)(now - vision_last_rx) > 100U && !nav_override && ReceivedPacketVision.mode != 0) {
      ReceivedPacketVision.mode = 0;
      Update_to_resolve();
    }
    if ((uint32_t)(now - last_state) >= 5U) {
      last_state = now;
#if USB_ATS_COMPAT_FEEDBACK_TX
      unified_usb_send_ats_feedback();
#else
#if USB_LEGACY_VISION_TX
      usb_send_vision();
#else
      unified_usb_send_state();
#endif
#endif
    }
#endif
    osDelay(1);
  }
}
static void reset(uint32_t now) {
    tick=now; rx_advance=0; inject_vision=inject_nav=0;
    nav_override=0; nav_last_rx=vision_last_rx=now;
    ReceivedPacketVision.mode=vision_cmd.mode=1;
    Navigation_speed_set.vx=Navigation_speed_set.vy=Navigation_speed_set.wz=3;
    resolved=0;
}
static void run(void) { if (!setjmp(stop_loop)) usb_task(0); }
static void check(const char *name, int ok) {
    printf("%s: %s\n",ok?"PASS":"FAIL",name);
    if (!ok) ++failures;
}
int main(void) {
    reset(1000); inject_vision=1; run();
    check("fresh vision, same tick",ReceivedPacketVision.mode==1 && resolved==0);
    reset(1000); inject_vision=1; rx_advance=1; run();
    check("fresh vision crosses 1ms tick",ReceivedPacketVision.mode==1 && resolved==0);
    reset(1000); inject_vision=1; rx_advance=25; run();
    check("fresh vision after processing/preemption delay",ReceivedPacketVision.mode==1 && resolved==0);
    reset(1000); inject_nav=1; rx_advance=1; run();
    check("fresh navigation crosses tick",nav_override==1 && ReceivedPacketVision.mode==2 && Navigation_speed_set.vx==3);
    reset(1000); vision_last_rx=900; run();
    check("vision age exactly 100ms remains valid",ReceivedPacketVision.mode==1 && resolved==0);
    reset(1000); vision_last_rx=899; run();
    check("vision age 101ms expires",ReceivedPacketVision.mode==0 && resolved==1);
    reset(1000); nav_override=1; nav_last_rx=900; vision_last_rx=800; ReceivedPacketVision.mode=2; run();
    check("navigation age exactly 100ms remains valid",nav_override==1 && ReceivedPacketVision.mode==2);
    reset(1000); nav_override=1; nav_last_rx=899; vision_last_rx=990; ReceivedPacketVision.mode=2; run();
    check("expired navigation falls back to fresh vision",nav_override==0 && ReceivedPacketVision.mode==1 && Navigation_speed_set.vx==0);
    reset(1000); nav_override=1; nav_last_rx=899; vision_last_rx=899; ReceivedPacketVision.mode=2; run();
    check("both command sources expired stop",nav_override==0 && ReceivedPacketVision.mode==0 && Navigation_speed_set.vx==0);
    reset(UINT32_MAX); inject_vision=1; rx_advance=1; run();
    check("fresh vision received across tick rollover",ReceivedPacketVision.mode==1 && resolved==0);
    reset(5); vision_last_rx=UINT32_MAX-15; run();
    check("21ms age across rollover stays valid",ReceivedPacketVision.mode==1 && resolved==0);
    reset(5); vision_last_rx=UINT32_MAX-95; run();
    check("101ms age across rollover expires",ReceivedPacketVision.mode==0 && resolved==1);
    return failures?1:0;
}
