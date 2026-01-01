
#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\data_stream\data_stream.h"
#include "command_parser.h"

#include "mirobotController.h"

#include <lcdgfx.h>
#undef max
#undef min

#include "src/XPT2046_Touchscreen/XPT2046_Touchscreen.h"
#include <lvgl.h>


// The parameters are  RST pin, BUS number, CS pin, DC pin, FREQ (0 means default), CLK pin, MOSI pin
DisplayILI9341_240x320x16_SPI display(3, { -1, 4, 5, 20000000, -1, -1 });
XPT2046_Touchscreen touchScreen(6);

#include "robot.h"
#include "interpolation.h"

TaskHandle_t HandleTaskControl;
TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskCOM;
TaskHandle_t HandleTaskCMD;

SemaphoreHandle_t ControlSemaphore;
SemaphoreHandle_t xMutex;

TimerHandle_t ControlTimer;
QueueHandle_t CMD_Queue;

constexpr std::size_t DOF = 6;

typedef enum {
  control_thetas = 0,
  control_position_orientation = 1,
  control_CMD = 2,
} control_mode_t;

typedef enum {
  mode_XYZ = 0,
  mode_RPY = 1,
} joystick_mode_t;


control_mode_t mode = control_thetas;
joystick_mode_t joystick_mode = mode_XYZ;

void setup(void) {

  delay(100);
  Serial.begin(250000);

  ControlSemaphore = xSemaphoreCreateBinary();
  CMD_Queue = xQueueCreate(30, sizeof(PoseCommand));
  xMutex = xSemaphoreCreateMutex();

  xTaskCreate(TaskControl,
              "Control",
              5000,
              NULL,
              tskIDLE_PRIORITY + 4,
              &HandleTaskControl);

  xTaskCreate(TaskUI,
              "UI",
              10000,
              NULL,
              tskIDLE_PRIORITY + 2,
              &HandleTaskUI);

  xTaskCreate(TaskCom,
              "Com",
              500,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCOM);


  xTaskCreate(TaskCMD,
              "CMD",
              2000,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskCMD);

  vTaskStartScheduler();
}

void loop(void) {
}


Matrix<DOF, 1> thetas = Matrix<DOF, 1>::Zero();
positionOrientation desired_effector_position_orientation;
positionOrientation feasible_effector_position_orientation;
positionOrientation effector_position_orientation;
bool IK_succ = true;

const float rate = 1.5;

#define SCREEN_COUNT 5
lv_obj_t *screens[SCREEN_COUNT];
int current_screen = 0;

lv_obj_t *btn_prev;
lv_obj_t *btn_next;
lv_obj_t *dd_mode;

void show_screen(int index) {
  current_screen = index % SCREEN_COUNT;
  if (current_screen < 0) current_screen = SCREEN_COUNT - 1;
  lv_obj_set_parent(btn_prev, screens[current_screen]);
  lv_obj_set_parent(btn_next, screens[current_screen]);
  lv_obj_set_parent(dd_mode, screens[current_screen]);
  lv_scr_load(screens[current_screen]);
}

void btn_next_event_cb(lv_event_t *e) {
  show_screen((current_screen + 1) % SCREEN_COUNT);
}

void btn_prev_event_cb(lv_event_t *e) {
  show_screen((current_screen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
}


void mode_dropdown_event_cb(lv_event_t *e) {
  lv_obj_t *dropdown = (lv_obj_t *)lv_event_get_target(e);
  uint16_t selected = lv_dropdown_get_selected(dropdown);
  mode = (control_mode_t)selected;
}

lv_obj_t *joy_base;
lv_obj_t *joy_knob;

void joy_knob_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);

  // Handle pressing
  if (code == LV_EVENT_PRESSING) {

    // Get base coordinates in screen space
    lv_area_t base_coords;
    lv_obj_get_coords(joy_base, &base_coords);

    // Center of base (screen coordinates)
    lv_coord_t cx = (base_coords.x1 + base_coords.x2) / 2;
    lv_coord_t cy = (base_coords.y1 + base_coords.y2) / 2;

    lv_coord_t joy_radius = (lv_obj_get_width(joy_base) - lv_obj_get_width(joy_knob)) / 2;
    lv_indev_t *indev = lv_indev_get_act();

    lv_point_t point;
    lv_indev_get_point(indev, &point);  // screen coordinates

    int dx = -(point.x - cx);
    int dy = -(point.y - cy);

    // Clamp to joystick radius
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > joy_radius) {
      dx = (int)((float)dx * (float)joy_radius / dist);
      dy = (int)((float)dy * (float)joy_radius / dist);
    }

    if (mode == control_position_orientation) {

      positionOrientation desired_effector_position_orientation_;

      xSemaphoreTake(xMutex, portMAX_DELAY);
      desired_effector_position_orientation_ = desired_effector_position_orientation;
      xSemaphoreGive(xMutex);


      if (joystick_mode == mode_XYZ) {
        desired_effector_position_orientation_.X += dx * rate * 3e-5;
        desired_effector_position_orientation_.Y += dy * rate * 3e-5;
      } else if (joystick_mode == mode_RPY) {
        desired_effector_position_orientation_.r += dx * rate * 1e-3;
        desired_effector_position_orientation_.p += dy * rate * 1e-3;
      }

      xSemaphoreTake(xMutex, portMAX_DELAY);
      desired_effector_position_orientation = desired_effector_position_orientation_;
      xSemaphoreGive(xMutex);
    }


    lv_obj_align_to(joy_knob, joy_base, LV_ALIGN_CENTER, -dx, -dy);
  }

  // Handle release
  else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    lv_obj_align_to(joy_knob, joy_base, LV_ALIGN_CENTER, 0, 0);
  }
}

lv_obj_t *lever_base;
lv_obj_t *lever_knob;

void lever_knob_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_PRESSING) {

    lv_area_t base_coords;
    lv_obj_get_coords(lever_base, &base_coords);
    lv_coord_t cy = (base_coords.y1 + base_coords.y2) / 2;
    lv_coord_t base_height = (lv_obj_get_height(lever_base) - lv_obj_get_height(lever_knob)) / 2;

    lv_indev_t *indev = lv_indev_get_act();

    lv_point_t point;
    lv_indev_get_point(indev, &point);

    int dy = -(point.y - cy);

    if (abs(dy) > base_height) {
      dy = (int)((float)dy * (float)(base_height) / abs(dy));
    }
    lv_obj_align_to(lever_knob, lever_base, LV_ALIGN_CENTER, 0, -dy);

    if (mode == control_position_orientation) {
      positionOrientation desired_effector_position_orientation_;

      xSemaphoreTake(xMutex, portMAX_DELAY);
      desired_effector_position_orientation_ = desired_effector_position_orientation;
      xSemaphoreGive(xMutex);


      if (joystick_mode == mode_XYZ) {
        desired_effector_position_orientation_.Z += dy * rate * 3e-5;
      } else if (joystick_mode == mode_RPY) {
        desired_effector_position_orientation_.y += dy * rate * 1e-3;
      }

      xSemaphoreTake(xMutex, portMAX_DELAY);
      desired_effector_position_orientation = desired_effector_position_orientation_;
      xSemaphoreGive(xMutex);
    }

  } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    lv_obj_align_to(lever_knob, lever_base, LV_ALIGN_CENTER, 0, 0);
  }
}


lv_obj_t *joint_sliders[DOF];

void joint_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int idx = reinterpret_cast<int>(lv_event_get_user_data(e));
  int value = lv_slider_get_value(slider);
  if (mode == control_thetas) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    thetas(idx) = (float)(value) / 180.0 * M_PI;
    xSemaphoreGive(xMutex);
  }
}

void xyz_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int idx = reinterpret_cast<int>(lv_event_get_user_data(e));
  int value = lv_slider_get_value(slider);


  if (mode == control_position_orientation) {


    positionOrientation desired_effector_position_orientation_;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    desired_effector_position_orientation_ = desired_effector_position_orientation;
    xSemaphoreGive(xMutex);

    if (idx == 1)
      desired_effector_position_orientation_.X = (float)(value) / 1000.0;
    else if (idx == 2)
      desired_effector_position_orientation_.Y = (float)(value) / 1000.0;
    else if (idx == 3)
      desired_effector_position_orientation_.Z = (float)(value) / 1000.0;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    desired_effector_position_orientation = desired_effector_position_orientation_;
    xSemaphoreGive(xMutex);
  }
}

void rpy_arc_event_cb(lv_event_t *e) {
  lv_obj_t *arc = (lv_obj_t *)lv_event_get_target(e);
  int idx = reinterpret_cast<int>(lv_event_get_user_data(e));
  int value = lv_arc_get_value(arc);

  if (mode == control_position_orientation) {

    positionOrientation desired_effector_position_orientation_;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    desired_effector_position_orientation_ = desired_effector_position_orientation;
    xSemaphoreGive(xMutex);

    if (idx == 1)
      desired_effector_position_orientation_.r = (float)(value) / 180.0 * M_PI;
    else if (idx == 2)
      desired_effector_position_orientation_.p = (float)(value) / 180.0 * M_PI;
    else if (idx == 3)
      desired_effector_position_orientation_.y = (float)(value) / 180.0 * M_PI;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    desired_effector_position_orientation = desired_effector_position_orientation_;
    xSemaphoreGive(xMutex);
  }
}

void ok_btn_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *mbox = (lv_obj_t *)lv_event_get_user_data(e);
  lv_msgbox_close(mbox);
}

void show_warning(const char *msg) {
  lv_obj_t *mbox = (lv_obj_t *)lv_msgbox_create(NULL); /* create empty msgbox */
  lv_obj_set_style_bg_opa(mbox, LV_OPA_COVER, 0);
  lv_msgbox_add_title(mbox, LV_SYMBOL_WARNING " Warning");
  lv_msgbox_add_text(mbox, msg);

  lv_obj_t *ok = lv_msgbox_add_footer_button(mbox, "OK"); /* returns the button */

  lv_obj_set_size(mbox, 180, 120);
  lv_obj_center(mbox);

  lv_obj_add_event_cb(ok, ok_btn_cb, LV_EVENT_CLICKED, mbox);

  lv_obj_center(mbox);
}

void radio_event_cb(lv_event_t *e) {
  lv_obj_t *cb = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *parent = (lv_obj_t *)lv_obj_get_parent(cb);


  uint32_t i;
  uint32_t child_cnt = lv_obj_get_child_cnt(parent);
  for (i = 0; i < child_cnt; i++) {
    lv_obj_t *child = lv_obj_get_child(parent, i);
    if (child != cb) lv_obj_clear_state(child, LV_STATE_CHECKED);
    else {
      if (i == 0)
        joystick_mode = mode_XYZ;
      else if (i == 1)
        joystick_mode = mode_RPY;
    }
  }

  lv_obj_add_state(cb, LV_STATE_CHECKED);
}


void TaskUI(void *pvParameters) {


#define TFT_HOR_RES 240
#define TFT_VER_RES 320

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 5 * (LV_COLOR_DEPTH / 8))
  uint32_t draw_buf[DRAW_BUF_SIZE / 4];

  display.begin();
  display.clear();

  touchScreen.begin();

  lv_init();
  lv_tick_set_cb(my_tick);

  lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  lv_display_set_flush_cb(disp, disp_flush);
  lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchpad_read);


  screens[0] = lv_obj_create(NULL);
  screens[1] = lv_obj_create(NULL);
  screens[2] = lv_obj_create(NULL);
  screens[3] = lv_obj_create(NULL);
  screens[4] = lv_obj_create(NULL);


  btn_prev = lv_btn_create(screens[0]);
  lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_LEFT, 20, -20);
  lv_obj_add_event_cb(btn_prev, btn_prev_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_prev = lv_label_create(btn_prev);
  lv_label_set_text(lbl_prev, "<<");
  lv_obj_center(lbl_prev);

  btn_next = lv_btn_create(screens[0]);
  lv_obj_align(btn_next, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
  lv_obj_add_event_cb(btn_next, btn_next_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_next = lv_label_create(btn_next);
  lv_label_set_text(lbl_next, ">>");
  lv_obj_center(lbl_next);

  dd_mode = lv_dropdown_create(screens[0]);
  lv_dropdown_set_options(dd_mode,
                          "Ang.\n"
                          "IK.\n"
                          "Cmd.");

  lv_obj_set_width(dd_mode, 85);
  lv_dropdown_set_selected(dd_mode, 0);

  lv_obj_align(dd_mode, LV_ALIGN_BOTTOM_MID, -5, -20);
  lv_obj_add_event_cb(dd_mode, mode_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  static lv_obj_t *label_x;
  static lv_obj_t *label_y;
  static lv_obj_t *label_z;

  label_x = lv_label_create(screens[0]);
  lv_label_set_text(label_x, "X=0.00");
  lv_obj_align(label_x, LV_ALIGN_TOP_LEFT, 10, 10);

  /* Create label for Y */
  label_y = lv_label_create(screens[0]);
  lv_label_set_text(label_y, "Y=0.00");
  lv_obj_align(label_y, LV_ALIGN_TOP_LEFT, 80, 10);

  /* Create label for Z */
  label_z = lv_label_create(screens[0]);
  lv_label_set_text(label_z, "Z=0.00");
  lv_obj_align(label_z, LV_ALIGN_TOP_LEFT, 170, 10);

  lv_obj_t *slider_x;
  lv_obj_t *slider_y;
  lv_obj_t *slider_z;

  slider_x = lv_slider_create(screens[0]);
  lv_obj_set_size(slider_x, 30, 180);
  lv_obj_align(slider_x, LV_ALIGN_CENTER, -75, -10);
  lv_slider_set_range(slider_x, -300, 300);
  lv_obj_add_event_cb(slider_x, xyz_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void *)1);

  lv_obj_t *lbl_slider_x = lv_label_create(screens[0]);
  lv_label_set_text(lbl_slider_x, "X");
  lv_obj_align_to(lbl_slider_x, slider_x, LV_ALIGN_OUT_TOP_MID, 0, -5);

  slider_y = lv_slider_create(screens[0]);
  lv_obj_set_size(slider_y, 30, 180);
  lv_obj_align(slider_y, LV_ALIGN_CENTER, 0, -10);
  lv_slider_set_range(slider_y, 0, 350);
  lv_obj_add_event_cb(slider_y, xyz_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void *)2);

  lv_obj_t *lbl_slider_y = lv_label_create(screens[0]);
  lv_label_set_text(lbl_slider_y, "Y");
  lv_obj_align_to(lbl_slider_y, slider_y, LV_ALIGN_OUT_TOP_MID, 0, -5);

  slider_z = lv_slider_create(screens[0]);
  lv_obj_set_size(slider_z, 30, 180);
  lv_obj_align(slider_z, LV_ALIGN_CENTER, 75, -10);
  lv_slider_set_range(slider_z, 0, 400);
  lv_obj_add_event_cb(slider_z, xyz_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void *)3);

  lv_obj_t *lbl_slider_z = lv_label_create(screens[0]);
  lv_label_set_text(lbl_slider_z, "Z");
  lv_obj_align_to(lbl_slider_z, slider_z, LV_ALIGN_OUT_TOP_MID, 0, -5);


  static lv_obj_t *label_roll;
  static lv_obj_t *label_pitch;
  static lv_obj_t *label_yaw;
  label_roll = lv_label_create(screens[1]);
  lv_label_set_text(label_roll, "r=0.00");
  lv_obj_align(label_roll, LV_ALIGN_TOP_LEFT, 10, 10);

  label_pitch = lv_label_create(screens[1]);
  lv_label_set_text(label_pitch, "p=0.00");
  lv_obj_align(label_pitch, LV_ALIGN_TOP_LEFT, 90, 10);

  label_yaw = lv_label_create(screens[1]);
  lv_label_set_text(label_yaw, "y=0.00");
  lv_obj_align(label_yaw, LV_ALIGN_TOP_LEFT, 170, 10);

  lv_obj_t *arc_roll;
  lv_obj_t *arc_pitch;
  lv_obj_t *arc_yaw;

  arc_roll = lv_arc_create(screens[1]);
  lv_obj_set_size(arc_roll, 70, 90);
  lv_obj_align(arc_roll, LV_ALIGN_CENTER, -80, -60);
  lv_arc_set_range(arc_roll, -180, 180);

  lv_obj_add_event_cb(arc_roll, rpy_arc_event_cb, LV_EVENT_VALUE_CHANGED, (void *)1);

  lv_obj_t *lbl_roll = lv_label_create(screens[1]);
  lv_label_set_text(lbl_roll, "Roll");
  lv_obj_align_to(lbl_roll, arc_roll, LV_ALIGN_OUT_BOTTOM_MID, 0, -30);

  arc_pitch = lv_arc_create(screens[1]);
  lv_obj_set_size(arc_pitch, 70, 90);
  lv_obj_align(arc_pitch, LV_ALIGN_CENTER, 0, 0);
  lv_arc_set_range(arc_pitch, -180, 180);

  lv_obj_add_event_cb(arc_pitch, rpy_arc_event_cb, LV_EVENT_VALUE_CHANGED, (void *)2);


  lv_obj_t *lbl_pitch = lv_label_create(screens[1]);
  lv_label_set_text(lbl_pitch, "Pitch");
  lv_obj_align_to(lbl_pitch, arc_pitch, LV_ALIGN_OUT_BOTTOM_MID, 0, -30);

  arc_yaw = lv_arc_create(screens[1]);
  lv_obj_set_size(arc_yaw, 70, 90);
  lv_obj_align(arc_yaw, LV_ALIGN_CENTER, 80, 60);
  lv_arc_set_range(arc_yaw, -180, 180);

  lv_obj_add_event_cb(arc_yaw, rpy_arc_event_cb, LV_EVENT_VALUE_CHANGED, (void *)3);

  lv_obj_t *lbl_yaw = lv_label_create(screens[1]);
  lv_label_set_text(lbl_yaw, "Yaw");
  lv_obj_align_to(lbl_yaw, arc_yaw, LV_ALIGN_OUT_BOTTOM_MID, 0, -30);


  for (int i = 0; i < DOF; i++) {
    joint_sliders[i] = lv_slider_create(screens[2]);
    lv_obj_set_size(joint_sliders[i], 20, 180);
    lv_obj_align(joint_sliders[i], LV_ALIGN_CENTER, -100 + i * 40, 10);
    lv_slider_set_range(joint_sliders[i], -100, 100);
    lv_obj_add_event_cb(joint_sliders[i], joint_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void *)i);

    lv_obj_t *lbl_slider = lv_label_create(screens[2]);
    char buf[8];
    sprintf(buf, "J %d", i + 1);  // label text
    lv_label_set_text(lbl_slider, buf);
    lv_obj_align_to(lbl_slider, joint_sliders[i], LV_ALIGN_OUT_TOP_MID, 0, -5);
  }

  joy_base = lv_obj_create(screens[3]);
  lv_obj_set_size(joy_base, 180, 180);
  lv_obj_set_style_radius(joy_base, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(joy_base, 2, 0);
  lv_obj_set_style_bg_color(joy_base, lv_color_hex(0x333333), 0);
  lv_obj_align(joy_base, LV_ALIGN_LEFT_MID, 5, -50);
  lv_obj_clear_flag(joy_base, LV_OBJ_FLAG_SCROLLABLE);

  joy_knob = lv_obj_create(joy_base);
  lv_obj_set_size(joy_knob, 40, 40);
  lv_obj_set_style_radius(joy_knob, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(joy_knob, lv_color_hex(0x00AAFF), 0);
  lv_obj_align(joy_knob, LV_ALIGN_CENTER, 0, 0);

  lv_obj_add_event_cb(joy_knob, joy_knob_event_cb, LV_EVENT_ALL, NULL);

  lever_base = lv_obj_create(screens[3]);
  lv_obj_set_size(lever_base, 40, 180);
  lv_obj_center(lever_base);
  lv_obj_set_style_radius(lever_base, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(lever_base, 2, 0);
  lv_obj_set_style_bg_color(lever_base, lv_color_hex(0x333333), 0);
  lv_obj_align(lever_base, LV_ALIGN_RIGHT_MID, -10, -50);
  lv_obj_clear_flag(lever_base, LV_OBJ_FLAG_SCROLLABLE);


  lever_knob = lv_obj_create(lever_base);
  lv_obj_set_size(lever_knob, 30, 35);
  lv_obj_set_style_radius(lever_knob, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(lever_knob, lv_color_hex(0x00AAFF), 0);
  lv_obj_align(lever_knob, LV_ALIGN_CENTER, 0, 0);  // start at bottom
  lv_obj_add_event_cb(lever_knob, lever_knob_event_cb, LV_EVENT_ALL, NULL);


  lv_obj_t *cont = lv_obj_create(screens[3]);
  lv_obj_set_size(cont, 180, 40);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(cont, 5, 0);
  lv_obj_align(cont, LV_ALIGN_BOTTOM_LEFT, 5, -70);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *cb1 = lv_checkbox_create(cont);
  lv_checkbox_set_text(cb1, "XYZ");
  lv_obj_add_event_cb(cb1, radio_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_state(cb1, LV_STATE_CHECKED);

  lv_obj_t *cb2 = lv_checkbox_create(cont);
  lv_checkbox_set_text(cb2, "RPY");
  lv_obj_add_event_cb(cb2, radio_event_cb, LV_EVENT_CLICKED, NULL);


  const int update_rate = 50;

  Matrix<DOF, 1> thetas_;
  positionOrientation desired_effector_position_orientation_;
  positionOrientation effector_position_orientation_;

  xSemaphoreTake(xMutex, portMAX_DELAY);


  desired_effector_position_orientation.X = 0.0;
  desired_effector_position_orientation.Y = 0.2;
  desired_effector_position_orientation.Z = 0.23;
  desired_effector_position_orientation.r = 0;
  desired_effector_position_orientation.p = 0;
  desired_effector_position_orientation.y = 0;

  xSemaphoreGive(xMutex);

  show_screen(0);

  while (1) {


    xSemaphoreTake(xMutex, portMAX_DELAY);
    effector_position_orientation_ = effector_position_orientation;
    xSemaphoreGive(xMutex);

    char buf[16];
    sprintf(buf, "X=%.1f", effector_position_orientation_.X * 1000.0);
    lv_label_set_text(label_x, buf);
    sprintf(buf, "Y=%.1f", effector_position_orientation_.Y * 1000.0);
    lv_label_set_text(label_y, buf);
    sprintf(buf, "Z=%.1f", effector_position_orientation_.Z * 1000.0);
    lv_label_set_text(label_z, buf);

    sprintf(buf, "r=%.1f", effector_position_orientation_.r / M_PI * 180.0);
    lv_label_set_text(label_roll, buf);
    sprintf(buf, "p=%.1f", effector_position_orientation_.p / M_PI * 180.0);
    lv_label_set_text(label_pitch, buf);
    sprintf(buf, "y=%.1f", effector_position_orientation_.y / M_PI * 180.0);
    lv_label_set_text(label_yaw, buf);



    if (mode == control_thetas || mode == control_CMD) {


      lv_arc_set_value(arc_roll, (int)(effector_position_orientation_.r / M_PI * 180.0));
      lv_arc_set_value(arc_pitch, (int)(effector_position_orientation_.p / M_PI * 180.0));
      lv_arc_set_value(arc_yaw, (int)(effector_position_orientation_.y / M_PI * 180.0));

      lv_slider_set_value(slider_x, (int)(effector_position_orientation_.X * 1000.0), 0);
      lv_slider_set_value(slider_y, (int)(effector_position_orientation_.Y * 1000.0), 0);
      lv_slider_set_value(slider_z, (int)(effector_position_orientation_.Z * 1000.0), 0);

    } else {


      xSemaphoreTake(xMutex, portMAX_DELAY);
      thetas_ = thetas;
      xSemaphoreGive(xMutex);


      for (int i = 0; i < DOF; i++) {
        lv_slider_set_value(joint_sliders[i], (int)(thetas_(i) / M_PI * 180.0), 0);
      }

      if (!IK_succ) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        desired_effector_position_orientation = feasible_effector_position_orientation;
        xSemaphoreGive(xMutex);
        lv_arc_set_value(arc_roll, (int)(desired_effector_position_orientation.r / M_PI * 180.0));
        lv_arc_set_value(arc_pitch, (int)(desired_effector_position_orientation.p / M_PI * 180.0));
        lv_arc_set_value(arc_yaw, (int)(desired_effector_position_orientation.y / M_PI * 180.0));

        lv_slider_set_value(slider_x, (int)(desired_effector_position_orientation.X * 1000.0), 0);
        lv_slider_set_value(slider_y, (int)(desired_effector_position_orientation.Y * 1000.0), 0);
        lv_slider_set_value(slider_z, (int)(desired_effector_position_orientation.Z * 1000.0), 0);
        show_warning("IK unfeasible");
      }
    }


    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(update_rate));
  }
}




void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {

  int32_t w = lv_area_get_width(area);
  int32_t h = lv_area_get_height(area);

  display.drawBuffer16(area->x1, area->y1, w, h, px_map);
  lv_display_flush_ready(disp);
}


void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {

  bool touched = touchScreen.touched();

  if (!touched) {
    data->state = LV_INDEV_STATE_RELEASED;
  } else {
    TS_Point p = touchScreen.getPoint();
    int y = map(p.x, 150, 3800, TFT_VER_RES - 1, 0);
    int x = map(p.y, 150, 3800, TFT_HOR_RES - 1, 0);

    data->state = LV_INDEV_STATE_PRESSED;

    data->point.x = x;
    data->point.y = y;
  }
}
uint32_t my_tick(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}



void TaskControl(void *pvParameters) {

  const float Ts = 0.1;

  HardwareSerial Serial1(PG9, PG14);
  Serial1.begin(115200);
  std::vector<actuator_parameters> mirobot_parameters;
  mirobot_parameters.resize(6);
  mirobot_parameters[0] = { -110.0 * PI / 180.0, 160.0 * PI / 180.0, 1.0, 0.0 };
  mirobot_parameters[1] = { -70.0 * PI / 180.0, 35.0 * PI / 180.0, -1.0, 0.0 };
  mirobot_parameters[2] = { -60.0 * PI / 180.0, 120.0 * PI / 180.0, -1.0, 0.0 };
  mirobot_parameters[3] = { -180.0 * PI / 180.0, 180.0 * PI / 180.0, 1.0, 0.0 };
  mirobot_parameters[4] = { -30.0 * PI / 180.0, 200.0 * PI / 180.0, -1.0, 0.0 };
  mirobot_parameters[5] = { -360.0 * PI / 180.0, 360.0 * PI / 180.0, 1.0, 0.0 };

  MirobotController mirobot_controller(&Serial1, mirobot_parameters);
  mirobot_controller.init();

  Matrix<3, 1> T[DOF];

/*
  T[0] << TODO
  T[1] << TODO
  T[2] << TODO
  T[3] << TODO
  T[4] << TODO
  T[5] << TODO
*/

  RotationZ rotZ_obj;
  RotationX rotX_obj;
  RotationY rotY_obj;
  const RotationMatrix *Rr[DOF] = { &rotZ_obj, &rotX_obj, &rotX_obj, &rotY_obj, &rotX_obj, &rotZ_obj };
  Robot::RobotKinematicsRotPtrs<DOF> robot_kinematics(Rr, T);
  Robot::RobotArm<DOF> robot(&robot_kinematics, &mirobot_controller);


  PoseInterpolator interpolator;

  ControlTimer = xTimerCreate("TimerCon",
                              pdMS_TO_TICKS(Ts * 1e3),
                              pdTRUE,
                              (void *)0,
                              vTimerControlCallback);

  xTimerStart(ControlTimer, 0);

  xSemaphoreTake(xMutex, portMAX_DELAY);
  xSemaphoreGive(xMutex);


  xSemaphoreTake(xMutex, portMAX_DELAY);
  robot.setJointAngles(thetas);
  xSemaphoreGive(xMutex);

  while (1) {

    xSemaphoreTake(ControlSemaphore, portMAX_DELAY);

    auto [effector_position, R] = robot.getCurrentEffectorPositionOrientation();
    xSemaphoreTake(xMutex, portMAX_DELAY);
    effector_position_orientation = positionOrientation(effector_position, R);
    xSemaphoreGive(xMutex);

    if (mode == control_thetas) {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      robot.setJointAngles(thetas);
      xSemaphoreGive(xMutex);
    } else if (mode == control_position_orientation) {

      xSemaphoreTake(xMutex, portMAX_DELAY);
      auto Rref = desired_effector_position_orientation.toRotationMatrix();
      auto Re = R * (Rref.transpose());
      double ori_error = (3.0 - Re.trace()) / 4.0;
      double pos_error = (effector_position_orientation.positionVector() - desired_effector_position_orientation.positionVector()).norm();

      if (pos_error < 0.01 && ori_error < 0.001) {
      }
      IK_succ = robot.goTo(desired_effector_position_orientation);
      thetas = robot.getJointAngles();

      if (IK_succ) feasible_effector_position_orientation = desired_effector_position_orientation;

      xSemaphoreGive(xMutex);

    } else if (mode == control_CMD) {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      PoseCommand cmd;
      if (interpolator.hasNext()) {
        IK_succ = robot.goTo(interpolator.next());
        if (!IK_succ) {
          interpolator.reset();
          xQueueReset(CMD_Queue);
        }
        thetas = robot.getJointAngles();
      } else if (xQueueReceive(CMD_Queue, &cmd, 0)) {
        positionOrientation target_effector_position_orientation(cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw);
        if (cmd.type == PoseCommand::Type::MOVE) {
          IK_succ = robot.goTo(target_effector_position_orientation);
          thetas = robot.getJointAngles();
        } else if (cmd.type == PoseCommand::Type::LINEAR) {
          interpolator.init(effector_position_orientation, target_effector_position_orientation, cmd.velocity, Ts);
        }
      }
      xSemaphoreGive(xMutex);
    }
  }
}
void vTimerControlCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(ControlSemaphore);
}


void TaskCMD(void *pvParameters) {

  int SERIAL_BUFFER_SIZE = 2048;
  char serialBuffer[SERIAL_BUFFER_SIZE];

  while (1) {
    if (mode == control_CMD) {
      int len = Serial.readBytesUntil('\n', serialBuffer, SERIAL_BUFFER_SIZE - 1);
      if (len > 0) {
        serialBuffer[len] = '\0';  // null-terminate
        PoseCommand cmd;
        ParseStatus status = CommandParser::parse(serialBuffer, cmd);

        if (status == ParseStatus::OK) {
          xQueueSend(CMD_Queue, &cmd, 0);
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}



void TaskCom(void *pvParameters) {
  data_stream_server_instance data_stream_serial;
  data_stream_server_init(&data_stream_serial, data_stream_data_read_impl, data_stream_data_write_impl, serial_stream_read_fcn, serial_stream_write_fcn, (void *)&Serial, get_timestamp_impl, NULL, NULL);

  while (1) {
    data_stream_server(&data_stream_serial);
  }
}

data_stream_return_t data_stream_data_read_impl(data_stream_id_t stream_id, uint8_t data_size, void *data_ptr, void *user_data) {

  if (stream_id == 1) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    memcpy(data_ptr, thetas.data(), sizeof(float) * DOF);
    xSemaphoreGive(xMutex);
    return data_stream_ok;
  } else if (stream_id == 2) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    memcpy(data_ptr, reinterpret_cast<void *>(&effector_position_orientation), sizeof(positionOrientation));
    xSemaphoreGive(xMutex);
    return data_stream_ok;
  } else {
    return data_stream_wrong_stream_params;
  }
}


data_stream_return_t data_stream_data_write_impl(data_stream_id_t stream_id, uint8_t data_size, const void *data_ptr, void *user_data) {

  return data_stream_wrong_stream_params;
}

data_stream_timestamp_t get_timestamp_impl(void *unused) {
  return millis() * 1e-3;
}


size_t serial_stream_write_fcn(void *handle, const void *buffer, size_t size, uint32_t timeout) {
  HardwareSerial *S = (HardwareSerial *)handle;
  size_t written = S->write((char *)buffer, size);
  S->flush();
  return written;
}
size_t serial_stream_read_fcn(void *handle, void *buffer, size_t size, uint32_t timeout) {
  HardwareSerial *S = (HardwareSerial *)handle;
  return S->readBytes((char *)buffer, size);
}
