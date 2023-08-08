
#define INPUT_MAX_LEN 8192



enum eCommands {
    SET_PINS = 0,
    SIG_SINE,
    SIG_SQUARE,
    SIG_RAMP,
    SET_FREQ,
    SET_ARBITRARY,
    SIG_ARBITRARY,
    CREATE_ARBITRARY,
    COMMANDS_LEN
};


char* shell_get_input(void);

void shell_run(void);

void shell_set_command_done(void);
