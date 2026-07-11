#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <gpiod.h>
#include <dirent.h>
#include <string.h>
#include <time.h>
#include <yaml-cpp/yaml.h>
#include <glob.h>
#include <linux/input.h>
#include "main.h"
#include "lvgl/lvgl.h"
#include "input.h"
#include "gsmenu/gs_system.h"

extern YAML::Node config;
extern lv_group_t *main_group;
extern lv_indev_t * indev_drv;


extern int dvr_enabled;

#ifdef USE_SIMULATOR
bool menu_active;
lv_timer_t * timer = NULL;
#endif
#ifndef USE_SIMULATOR
extern bool menu_active;
#define MAX_GPIO_BUTTONS 6  // Adjust based on your hardware
#define DEBOUNCE_DELAY_MS 50 // Debounce delay in milliseconds
#define INITIAL_REPEAT_DELAY_MS 500  // Time before repeat starts
#define REPEAT_RATE_MS 100           // Time between repeated events

// gpio_button_t structure
typedef struct {
    const char *name;        // Button name ("up", "down", etc.)
    int pin_number;          // Physical pin number from YAML
    const char *chip_name;   // GPIO chip path (e.g., "/dev/gpiochip0")
    const char *chip_label;  // GPIO chip label (e.g., "gpiochip3")
    int line_num;            // GPIO line number
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int last_state;
    long last_time;
    long repeat_time;
    bool is_holding;
    bool long_press_sent;
} gpio_button_t;

// Global array of GPIO buttons
gpio_button_t gpio_buttons[MAX_GPIO_BUTTONS] = {0};
// 0 = pressed (active-low, pull-up wiring); 1 = pressed (active-high, pull-down wiring)
static int gpio_pressed_state = 1;
#endif

extern lv_obj_t * pp_menu_screen;

// Global or static variable to store the next key state
static lv_key_t next_key = LV_KEY_END;  // Default to no key
static bool next_key_pressed = false;    // Indicates if the next key should be pressed or released
gsmenu_control_mode_t control_mode = GSMENU_CONTROL_MODE_NAV;

extern uint64_t gtotal_tunnel_data;
void simulate_traffic(lv_timer_t *t)
{
    gtotal_tunnel_data++;
}

#ifndef USE_SIMULATOR
// Function to find GPIO chip and line for a given pin number
bool find_gpio_mapping(int pin, const char** chip_name, int* line_num) {
    glob_t globbuf;
    struct gpiod_chip *chip = NULL;
    bool found = false;

    if (glob("/dev/gpiochip*", 0, NULL, &globbuf) != 0) {
        perror("Failed to find GPIO chips");
        return false;
    }

    for (size_t i = 0; i < globbuf.gl_pathc && !found; i++) {
        chip = gpiod_chip_open(globbuf.gl_pathv[i]);
        if (!chip) continue;

        // Check chip label first
        const char *label = gpiod_chip_label(chip);
        if (label) {
            // If we were looking for a specific chip label, we'd check here
        }

        // For libgpiod v1.x
        int num_lines = gpiod_chip_num_lines(chip);
        for (int offset = 0; offset < num_lines && !found; offset++) {
            struct gpiod_line *line = gpiod_chip_get_line(chip, offset);
            if (!line) continue;

            const char *name = gpiod_line_name(line);
            if (name) {
                int extracted_pin = 0;
                if (sscanf(name, "PIN_%d", &extracted_pin) == 1 || 
                    sscanf(name, "GPIO%d", &extracted_pin) == 1 ||
                    sscanf(name, "%d", &extracted_pin) == 1) {
                    if (extracted_pin == pin) {
                        *chip_name = strdup(globbuf.gl_pathv[i]);
                        *line_num = offset;
                        found = true;
                    }
                }
            }
            gpiod_line_release(line);
        }
        gpiod_chip_close(chip);
    }
    globfree(&globbuf);
    return found;
}

void init_button_from_config(YAML::Node& gpio_config, const char* button_name, int& button_index) {
    if (!gpio_config[button_name] || gpio_config[button_name].IsNull()) {
        printf("Omitting GPIO mapping for button %s\n", button_name);
        return;
    }

    gpio_buttons[button_index].name = button_name;

    // Check if the button config is a simple pin number or a map
    if (gpio_config[button_name].IsScalar()) {
        // Simple format: just a pin number
        gpio_buttons[button_index].pin_number = gpio_config[button_name].as<int>();
        if (!find_gpio_mapping(gpio_buttons[button_index].pin_number, 
                             &gpio_buttons[button_index].chip_name,
                             &gpio_buttons[button_index].line_num)) {
            fprintf(stderr, "Failed to find GPIO mapping for pin %d (%s)\n", 
                    gpio_buttons[button_index].pin_number, button_name);
            return;
        }
    } else {
        // Complex format: chip and pin specified
        YAML::Node button_config = gpio_config[button_name];
        if (!button_config["chip"] || !button_config["pin"]) {
            fprintf(stderr, "Invalid GPIO config for button %s - missing chip or pin\n", button_name);
            return;
        }

        gpio_buttons[button_index].chip_label = strdup(button_config["chip"].as<std::string>().c_str());
        gpio_buttons[button_index].pin_number = button_config["pin"].as<int>();
        
        // Construct chip path from label
        std::string chip_path = "/dev/" + std::string(gpio_buttons[button_index].chip_label);
        gpio_buttons[button_index].chip_name = strdup(chip_path.c_str());
        
        // For direct chip/pin mapping, we can use the pin number directly as line_num
        gpio_buttons[button_index].line_num = gpio_buttons[button_index].pin_number;
    }

    button_index++;
}

void init_gpio_buttons_from_config(YAML::Node& config) {
    YAML::Node gpio_config = config["gsmenu"]["gpio"];
    int button_index = 0;

    if (gpio_config["active_low"] && gpio_config["active_low"].as<bool>())
        gpio_pressed_state = 0;

    init_button_from_config(gpio_config, "up", button_index);
    init_button_from_config(gpio_config, "down", button_index);
    init_button_from_config(gpio_config, "left", button_index);
    init_button_from_config(gpio_config, "right", button_index);
    init_button_from_config(gpio_config, "center", button_index);
    init_button_from_config(gpio_config, "rec", button_index);
}

// Function to initialize GPIO buttons
void setup_gpio(YAML::Node& config) {
    // Initialize buttons from config first
    init_gpio_buttons_from_config(config);
    
    // Then setup the GPIO lines
    for (size_t i = 0; i < sizeof(gpio_buttons) / sizeof(gpio_buttons[0]); i++) {
        if (gpio_buttons[i].chip_name == NULL) continue;
        
        gpio_buttons[i].chip = gpiod_chip_open(gpio_buttons[i].chip_name);
        if (!gpio_buttons[i].chip) {
            perror("Failed to open GPIO chip");
            continue;
        }

        gpio_buttons[i].line = gpiod_chip_get_line(gpio_buttons[i].chip, gpio_buttons[i].line_num);
        if (!gpio_buttons[i].line) {
            perror("Failed to get GPIO line");
            gpiod_chip_close(gpio_buttons[i].chip);
            continue;
        }

        // Create the consumer name with "pixelpilot_" prefix
        char consumer_name[32];
        snprintf(consumer_name, sizeof(consumer_name), "pixelpilot_%s", gpio_buttons[i].name);

        if (gpiod_line_request_input(gpio_buttons[i].line, consumer_name) < 0) {
            perror("Failed to request GPIO input");
            gpiod_chip_close(gpio_buttons[i].chip);
            gpio_buttons[i].chip = NULL;
            gpio_buttons[i].line = NULL;
        }
    }
}

static void handle_char_input(char c); // forward declaration

// evdev keyboard support
static int evdev_fd = -1;

static void init_evdev_keyboard() {
    // Try to find a keyboard in /dev/input/by-id first
    glob_t gl;
    if (glob("/dev/input/by-id/*-event-kbd", 0, NULL, &gl) == 0 && gl.gl_pathc > 0) {
        evdev_fd = open(gl.gl_pathv[0], O_RDONLY | O_NONBLOCK);
        if (evdev_fd >= 0)
            fprintf(stdout, "evdev keyboard: %s\n", gl.gl_pathv[0]);
    }
    globfree(&gl);
    // Fallback: scan /dev/input/event* for a device with KEY_W
    if (evdev_fd < 0) {
        for (int i = 0; i < 32; i++) {
            char path[32];
            snprintf(path, sizeof(path), "/dev/input/event%d", i);
            int fd = open(path, O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;
            uint8_t evbits[EV_MAX/8+1] = {0};
            uint8_t keybits[KEY_MAX/8+1] = {0};
            ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits);
            if (evbits[EV_KEY/8] & (1 << (EV_KEY%8))) {
                ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
                if (keybits[KEY_W/8] & (1 << (KEY_W%8))) {
                    evdev_fd = fd;
                    fprintf(stdout, "evdev keyboard fallback: %s\n", path);
                    break;
                }
            }
            close(fd);
        }
    }
    if (evdev_fd < 0)
        fprintf(stderr, "evdev keyboard: no keyboard found, falling back to stdin\n");
}

static void handle_evdev_input() {
    if (evdev_fd < 0) return;
    struct input_event ev;
    while (read(evdev_fd, &ev, sizeof(ev)) == sizeof(ev)) {
        if (ev.type != EV_KEY || ev.value != 1) continue; // only key-down
        char c = '\0';
        switch (ev.code) {
            case KEY_W: case KEY_UP:    c = 'w'; break;
            case KEY_S: case KEY_DOWN:  c = 's'; break;
            case KEY_A: case KEY_LEFT:  c = 'a'; break;
            case KEY_D: case KEY_RIGHT: c = 'd'; break;
            case KEY_ENTER:             c = '\n'; break;
            case KEY_Q:                 c = 'q'; break;
            case KEY_T:                 c = 't'; break;
            default: break;
        }
        if (c) handle_char_input(c);
    }
}

void toggle_screen(void); // defined below; used by send_long_press_event above its definition

// 'center'-long is the "open / cancel" gesture:
//   - menu closed            -> open it
//   - menu open, browsing    -> LV_KEY_HOME (same effect as 'left' in NAV
//                                mode below -- left is the day-to-day way
//                                to go back now, this is kept as an
//                                always-available alternative, and is the
//                                spot reserved for a future "discard
//                                changes made on this page" gesture)
//   - menu open, editing a   -> LV_KEY_ESC (existing per-widget
//     value (slider/dropdown/   LV_EVENT_CANCEL handlers, e.g.
//     keyboard)                 slider_event_cb / video_scale_revert_cb,
//                                already capture-on-entry and revert-on-ESC;
//                                this just wires a button to send it)
void send_long_press_event(size_t button_index) {
    if (strcmp(gpio_buttons[button_index].name, "center") != 0) return;

    if (!menu_active) {
        toggle_screen();
        printf("GPIO Long Press: center (opening menu)\n");
        return;
    }

    if (control_mode == GSMENU_CONTROL_MODE_NAV) {
        next_key = LV_KEY_HOME;
        printf("GPIO Long Press: center (back/close menu)\n");
    } else {
        next_key = LV_KEY_ESC;
        printf("GPIO Long Press: center (cancel edit)\n");
    }
    next_key_pressed = true;
}

void send_button_event(size_t button_index) {
    if (gpio_buttons[button_index].name == NULL) return;

    // While the menu is closed: 'rec' keeps its own function below;
    // left/right switch streams (no-ops if the multistream switcher isn't
    // running -- see switch_to_next_stream/switch_to_prev_stream in
    // main.cpp); up/down are reserved, no function yet. None of these fall
    // through to the NAV-mode branch below, which only makes sense once the
    // menu is actually open. Opening the menu itself is exclusively a
    // center-long-press gesture (see send_long_press_event).
    if (!menu_active) {
        if (strcmp(gpio_buttons[button_index].name, "left") == 0) {
            switch_to_prev_stream();
            return;
        }
        if (strcmp(gpio_buttons[button_index].name, "right") == 0) {
            switch_to_next_stream();
            return;
        }
        if (strcmp(gpio_buttons[button_index].name, "rec") != 0) {
            return; // up/down: reserved, no function yet
        }
        // 'rec' falls through to the NAV-mode switch below.
    }

    // Adjust for control_mode
    switch (control_mode) {
        case GSMENU_CONTROL_MODE_NAV:
            // up/down move through the focus chain (the only item-to-item
            // navigation now -- this menu's layout is a single chain per
            // page, not a 2D grid, so one axis is enough and frees left/right
            // for level navigation instead). right and center-short both
            // enter/select the focused item (a submenu visually appears to
            // the right of its parent, so right-to-enter reads naturally;
            // center stays as an equivalent for one-handed use). left goes
            // back one level (repeatable -- same LV_KEY_HOME handlers used
            // by center-long, see generic_back_event_handler/ui.c's
            // main-group HOME handler -- closes the menu entirely once
            // already at the top level).
            if (strcmp(gpio_buttons[button_index].name, "up") == 0) {
                next_key = LV_KEY_PREV;
            }
            else if (strcmp(gpio_buttons[button_index].name, "down") == 0) {
                next_key = LV_KEY_NEXT;
            }
            else if (strcmp(gpio_buttons[button_index].name, "right") == 0 ||
                     strcmp(gpio_buttons[button_index].name, "center") == 0) {
                next_key = LV_KEY_ENTER;
            }
            else if (strcmp(gpio_buttons[button_index].name, "left") == 0) {
                next_key = LV_KEY_HOME;
            }
            else if (strcmp(gpio_buttons[button_index].name, "rec") == 0) {
                #ifdef USE_SIMULATOR
                                dvr_enabled ^= 1;
                #endif
                            toggle_rec_enabled();
            }
            break;

        case GSMENU_CONTROL_MODE_EDIT: // dropdown: cycle options
            if (strcmp(gpio_buttons[button_index].name, "up") == 0) {
                next_key = LV_KEY_UP;
            }
            else if (strcmp(gpio_buttons[button_index].name, "down") == 0) {
                next_key = LV_KEY_DOWN;
            }
            else if (strcmp(gpio_buttons[button_index].name, "center") == 0) {
                next_key = LV_KEY_ENTER;
            }
            // left/right unbound here -- ESC moved to center-long, and
            // dropdown cycling is up/down-only.
            break;

        case GSMENU_CONTROL_MODE_SLIDER:
            if (strcmp(gpio_buttons[button_index].name, "left") == 0) {
                next_key = LV_KEY_LEFT;  // decrease
            }
            else if (strcmp(gpio_buttons[button_index].name, "right") == 0) {
                next_key = LV_KEY_RIGHT; // increase
            }
            else if (strcmp(gpio_buttons[button_index].name, "center") == 0) {
                next_key = LV_KEY_ENTER; // confirm, exit to NAV
            }
            // up/down unbound here -- slider adjustment is left/right now;
            // ESC (cancel/revert) moved to center-long.
            break;

        case GSMENU_CONTROL_MODE_KEYBOARD:
            if (strcmp(gpio_buttons[button_index].name, "up") == 0) {
                next_key = LV_KEY_UP;
            } 
            else if (strcmp(gpio_buttons[button_index].name, "down") == 0) {
                next_key = LV_KEY_DOWN;
            }
            else if (strcmp(gpio_buttons[button_index].name, "left") == 0) {
                next_key = LV_KEY_LEFT;
            }
            else if (strcmp(gpio_buttons[button_index].name, "right") == 0) {
                next_key = LV_KEY_RIGHT;
            }
            else if (strcmp(gpio_buttons[button_index].name, "center") == 0) {
                next_key = LV_KEY_ENTER;
            }
            break;
            
        default:
            break;
    }
    
    if (next_key != LV_KEY_END) {
        next_key_pressed = true;
        printf("GPIO %s: %s (Pin: %d, Chip: %s)\n", 
               gpio_buttons[button_index].is_holding ? "Holding" : "Pressed", 
               gpio_buttons[button_index].name,
               gpio_buttons[button_index].pin_number,
               gpio_buttons[button_index].chip_name);
    }
}

void handle_gpio_input(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long current_time = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    
    for (size_t i = 0; i < sizeof(gpio_buttons) / sizeof(gpio_buttons[0]); i++) {
        if (gpio_buttons[i].chip && gpio_buttons[i].line) {
            int current_state = gpiod_line_get_value(gpio_buttons[i].line);
            
            // Check for state change (with debounce)
            if (current_state != gpio_buttons[i].last_state &&
                (current_time - gpio_buttons[i].last_time) > DEBOUNCE_DELAY_MS) {
                
                gpio_buttons[i].last_state = current_state;
                gpio_buttons[i].last_time = current_time;
                
                if (current_state == gpio_pressed_state) { // Button pressed
                    gpio_buttons[i].is_holding = true;
                    gpio_buttons[i].long_press_sent = false;
                    gpio_buttons[i].repeat_time = current_time + INITIAL_REPEAT_DELAY_MS;
                    
                    // Fire event immediately for all buttons EXCEPT 'right', 'left'
                    // and 'center'. For those, we wait to see if it's a short or
                    // long press.
                    if (strcmp(gpio_buttons[i].name, "right") != 0 &&
                        strcmp(gpio_buttons[i].name, "left") != 0 &&
                        strcmp(gpio_buttons[i].name, "center") != 0) {
                        send_button_event(i);
                    }
                } else { // Button released
                    gpio_buttons[i].is_holding = false;

                    // If 'right'/'left'/'center' was released without a long press, send the normal event now.
                    if ((strcmp(gpio_buttons[i].name, "right") == 0 ||
                         strcmp(gpio_buttons[i].name, "left") == 0 ||
                         strcmp(gpio_buttons[i].name, "center") == 0) &&
                        !gpio_buttons[i].long_press_sent) {
                        send_button_event(i);
                    } else {
                        // For all other buttons, or for a long-pressed 'right' button,
                        // just send a generic release to LVGL.
                        next_key_pressed = false;
                    }
                }
            }
            
            // LOGIC FOR HELD BUTTONS (LONG PRESS / REPEAT) ---
            if (gpio_buttons[i].is_holding && current_state == gpio_pressed_state &&
                current_time >= gpio_buttons[i].repeat_time) {
                
                // Special long-press handling for 'right', 'left' and 'center'
                if (strcmp(gpio_buttons[i].name, "right") == 0 ||
                    strcmp(gpio_buttons[i].name, "left") == 0 ||
                    strcmp(gpio_buttons[i].name, "center") == 0) {
                    if (!gpio_buttons[i].long_press_sent) {
                        send_long_press_event(i);
                        gpio_buttons[i].long_press_sent = true;
                    }
                }
                else {
                    // Standard repeat for all other buttons
                    send_button_event(i);
                    gpio_buttons[i].repeat_time = current_time + REPEAT_RATE_MS;
                }
            }
        }
    }
}


// Cleanup function for GPIO
void cleanup_gpio(void) {
    for (int i = 0; i < MAX_GPIO_BUTTONS; i++) {
        if (gpio_buttons[i].chip) {
            gpiod_chip_close(gpio_buttons[i].chip);
            gpio_buttons[i].chip = NULL;
            gpio_buttons[i].line = NULL;
        }
        if (gpio_buttons[i].chip_name) {
            free((void*)gpio_buttons[i].chip_name);
            gpio_buttons[i].chip_name = NULL;
        }
        if (gpio_buttons[i].chip_label) {
            free((void*)gpio_buttons[i].chip_label);
            gpio_buttons[i].chip_label = NULL;
        }
    }
}
#endif

// Function to make stdin non-blocking
void set_stdin_nonblock(void) {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    // Disable canonical mode and echo
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

// Function to restore terminal settings
void restore_stdin(void) {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

void toggle_screen(void) {
    if( ! menu_active ) {
        lv_scr_load(pp_menu_screen);
        lv_indev_set_group(indev_drv,main_group);
        lv_obj_invalidate(pp_menu_screen);
        menu_active = true;
    }
}

// Handle WASD input and convert to LVGL key codes
void handle_keyboard_input(void) {
    char c;
    if (read(STDIN_FILENO, &c, 1) > 0) {
        handle_char_input(c);
    }
    handle_evdev_input();
}

static void handle_char_input(char c) {
        switch(c) {
            case 'w':
            case 'W':
                switch (control_mode)
                {
                case GSMENU_CONTROL_MODE_NAV:
                    next_key = LV_KEY_PREV;
                    break;
                case GSMENU_CONTROL_MODE_SLIDER:
                    next_key = LV_KEY_RIGHT;
                    break;
                case GSMENU_CONTROL_MODE_EDIT:
                    next_key = LV_KEY_UP;
                    break;
                case GSMENU_CONTROL_MODE_KEYBOARD:
                    next_key = LV_KEY_UP;
                    break;
                default:
                    break;
                }
                next_key_pressed = true;
                printf("Up\n");
                break;
            case 's':
            case 'S':
                switch (control_mode)
                {
                case GSMENU_CONTROL_MODE_SLIDER:
                    next_key = LV_KEY_LEFT;
                    break;
                case GSMENU_CONTROL_MODE_NAV:
                    next_key = LV_KEY_NEXT;
                    break;
                case GSMENU_CONTROL_MODE_EDIT:
                    next_key = LV_KEY_DOWN;
                    break;
                case GSMENU_CONTROL_MODE_KEYBOARD:
                    next_key = LV_KEY_DOWN;
                    break;
                default:
                    break;
                } 
                next_key_pressed = true;
                printf("Down\n");
                break;
            case 'a':
            case 'A':
#ifndef USE_SIMULATOR
                if (!menu_active) { switch_to_prev_stream(); break; }
#endif
                switch (control_mode)
                {
                case GSMENU_CONTROL_MODE_SLIDER:
                case GSMENU_CONTROL_MODE_EDIT:
                    next_key = LV_KEY_ESC;
                    break;
                    break;
                case GSMENU_CONTROL_MODE_NAV:
                    next_key = LV_KEY_HOME;
                    break;
                case GSMENU_CONTROL_MODE_KEYBOARD:
                    next_key = LV_KEY_LEFT;
                    break;
                default:
                    break;
                }
                next_key_pressed = true;
                printf("Left\n");
                break;
            case 'd':
            case 'D':
#ifndef USE_SIMULATOR
                if (!menu_active) { switch_to_next_stream(); break; }
#endif
                switch (control_mode)
                {
                case GSMENU_CONTROL_MODE_NAV:
                    next_key = LV_KEY_ENTER;
                    break;
                case GSMENU_CONTROL_MODE_SLIDER:
                    next_key = LV_KEY_ENTER;
                    break;
                case GSMENU_CONTROL_MODE_EDIT:
                    next_key = LV_KEY_ENTER;
                    break;
                case GSMENU_CONTROL_MODE_KEYBOARD:
                    next_key = LV_KEY_RIGHT;
                    break;
                default:
                    break;
                }
                next_key_pressed = true;
                printf("Right\n");
                break;
            case '\n':
                next_key = LV_KEY_ENTER;
                next_key_pressed = true;
                printf("Enter\n");
                break;
#ifdef USE_SIMULATOR
            case 't':
            case 'T':
                if (timer) {
                    lv_timer_delete(timer);
                    timer = NULL;
                }
                else
                    timer = lv_timer_create(simulate_traffic, 50, NULL);
                break;
#endif
            case 'q':
            case 'Q':
                raise(SIGINT);
                break;
            case 'm':
            case 'M':
                // Keyboard has no hold-duration concept, so there's no
                // equivalent to center-long-press otherwise -- this directly
                // calls what that gesture calls, for testing the menu
                // open/back/cancel path via keyboard on a dev machine
                // without needing real GPIO hardware.
#ifndef USE_SIMULATOR
                if (!menu_active) toggle_screen();
                else if (control_mode == GSMENU_CONTROL_MODE_NAV) { next_key = LV_KEY_HOME; next_key_pressed = true; }
                else { next_key = LV_KEY_ESC; next_key_pressed = true; }
#endif
                break;
        }
}

// Custom function to simulate keyboard input
static void virtual_keyboard_read(lv_indev_t * indev, lv_indev_data_t * data) {
    static bool key_sent = false;  // Track if a key event was sent

#ifndef USE_SIMULATOR
    handle_gpio_input(); // Check GPIO state separately from keyboard input
#endif

    if (next_key != LV_KEY_END) {
        data->key = next_key;
        data->state = next_key_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

        next_key_pressed = !next_key_pressed;  // Toggle state

        // Menu opening is now exclusively via center-long-press (see
        // send_long_press_event) -- no more implicit "any non-ENTER key
        // wakes the menu" side effect, so up/down/left/right are free to
        // mean something else entirely while the menu is closed.

        if (!next_key_pressed) {
            next_key = LV_KEY_END;  // Reset key after release event
        }

        key_sent = true;  // Mark that a key was sent
    } else if (key_sent) {
        data->state = LV_INDEV_STATE_REL;  // Ensure release event is sent
        key_sent = false;  // Reset the flag
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// Function to create the virtual keyboard
lv_indev_t * create_virtual_keyboard() {

    set_stdin_nonblock(); // setup keyboard input from stdin
#ifndef USE_SIMULATOR
    init_evdev_keyboard();       // Initialize evdev keyboard
    setup_gpio(config);          // Initialize GPIO
#endif
    lv_indev_t * indev_drv = lv_indev_create();
    lv_indev_set_type(indev_drv, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(indev_drv, virtual_keyboard_read);

    lv_indev_enable(indev_drv, true);

    return indev_drv;
}
