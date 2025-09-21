/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                    Soldering Station v 1.6.1              */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
 
#include <EEPROM.h>
#include <CyberLib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
/* it's fuckin' magic! */
#if 1
__asm volatile ("nop");
#endif
 
/* Options */
 
/* если вам нужно создать их диаграмму, то раскомментируйте ее */
//#define NEED_GRAPH
 
/* если используется не rail-to-rail OPA, то раскомментируйте ее */
//#define LM358
 
/* если используется пассивный busser, то раскомментируйте его */
//#define PASSIVE_BUZZER
 
/* если вы хотите отключить дополнительную защиту, то прокомментируйте это */
#define HA_ADV_PROT_ON
#define S_ADV_PROT_ON
 
/* if used LCD 1602, then uncomment it */
//#define LCD_1602
 
/* Паяльник - если вы хотите удалить прыжки цифр, то раскомментируйте его */
//#define SOLDER_DIGIT_JUMPS_REMOVE
 
/* Фен - если вы хотите удалить прыжки цифр, тогда раскомментируйте его */
//#define HOTAIR_DIGIT_JUMPS_REMOVE
 
/* если вы хотите мягкий старт паяльника, тогда раскомментируйте его */
//#define SOLDER_SOFT_START
 
/* если вы хотите активировать паяльник, таймер только на стенде, тогда раскомментируйте его */
#define SOLDER_TIMER_ON_STAND
 
/*если используется ENCODER, тогда раскомментируйте его */
#define ENCODER 
 
/* End options */
 
 
#define FIRMWARE_VERSION "1.6.1enc"
 
#ifdef LCD_1602
LiquidCrystal_I2C lcd(0x27, 16, 2);
#else
LiquidCrystal_I2C lcd(0x27, 20, 4);
#endif
 
 
/* Символ градуса */
uint8_t degree[8] = {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
};
 
/* Символ стрелки */
uint8_t arrow[8] =  {
    B00000,
    B00100,
    B00010,
    B11111,
    B00010,
    B00100,
    B00000,
    B00000
};
 
/* селектор режимов */
#define  modeSolder 1
#define  modeHotAir 2
#define  modeFanPWM 3
 
boolean need_S_countdown = false;
byte selected_Mode = modeSolder;
byte HA_countdown = 1;
byte S_countdown = 1;
uint16_t HA_sleeptime = 10;
uint16_t S_sleeptime = 10;
uint16_t Graph_count = 0;
 
#ifdef ENCODER
uint32_t Enc_ButtonPressTime = 0;
boolean Enc_ButtonState;
#else
uint32_t UPbuttonPressTime = 0;
uint32_t DWNbuttonPressTime = 0;
boolean UPbuttonState, DWNbuttonState,
#endif
 
uint32_t SONbuttonPressTime = 0;
uint32_t HAONbuttonPressTime = 0;
boolean SONbuttonState, HAONbuttonState;
 
byte Count;
uint16_t Duration, Interval;
 
char bspace[ ] = "    ";
 
#define pinBuzzer 12
#define pinFanPwm 6
#define pinSolderPwm 5
 
#define min_solder_temp 100
#define max_solder_temp 400
#define min_hotair_temp 20.0
#define max_hotair_temp 450.0
#define min_rpm 30
#define max_rpm 100
#define default_temp 280
#define default_rpm 50
 
/* Кнопки */
#define sw_HA 10
#define sw_S 9
#define bt_SON 13
#define bt_HAON 14
/*Енкодер*/
#ifdef ENCODER
#define e_button 15 //pin A1 button
#define e_dir 16    //pin A2 enc_a
#define e_step 17   //pin A3 enc_b
#else
#define bt_Sel 15   //pin A1
#define bt_Up 16    //pin A2
#define bt_Dwn 17   //pin A3
#endif
 
/* Bounce killers */
Bounce swHotAir = Bounce();
Bounce swSolder = Bounce();
Bounce SolderOnButton = Bounce();
Bounce HotAirOnButton = Bounce();
 
#ifdef ENCODER
Bounce Enc_Button = Bounce();
#else
Bounce SelButton = Bounce();
Bounce UpButton = Bounce();
Bounce DwnButton = Bounce();
#endif
 
/* Hot Air */
 
/* states */
#define st_stop 0
#define st_work 1
#define st_pause 2
#define st_protection 3
//#define st_lowpower 4
 
byte hotair_state = st_stop;
 
volatile uint16_t ots = 9990;
volatile float HAPower = 0.0;
uint16_t GetHotAirT = 0;
uint16_t SetHotAirT = 100;
byte SetHotAirRPM = 100;
boolean HA_temp_stable = false;
boolean need_Cooling = true;
boolean scr_blink = false;
 
byte ha_error = 0;
boolean HA_prot_beep = false;
boolean ha_f1 = false;
boolean ha_f2 = false;
uint32_t prevHAcontrol;
 
/* HA PI regulator */
#define Kp 1.0
#define Ki 0.007 //0.003 //0.005 //0.05
int integral = 0;
 
/* Soldering iron */
uint16_t GetSolderT = 0;
uint16_t SetSolderT = 100;
boolean S_temp_stable = false;
boolean SolderON = false;
boolean SolderProtect = false;
int SPower = 0;
//byte solder_state = st_stop;
 
byte s_error = 0;
uint32_t prevScontrol;
boolean S_prot_beep = false;
boolean s_f1 = false;
boolean s_f2 = false;
 
/* Solder P regulator */
#define sKp 50
 
uint16_t last_HotAirT, last_SolderT;
byte last_RPM;
 
 
/********************************************* ОСНОВНЫЕ ПРОЦЕДУРЫ *********************************************/
 
void setup() {
    //ADC change speedup
    //ADCSRA &= ~(1 <<ADPS2) | (1 <<ADPS1) | (1 <<ADPS0); // reset default divider 128
    //ADCSRA |= 1 <<ADPS2; // set div to 16 (1MHz)
    //ADCSRA |= 1 <<ADPS1; // set div to 64 (250kHz)
 
    pinMode(3, INPUT_PULLUP); //Zero cross pin
    D5_Out; //pinSolder (нагрев)
    D5_Low;
    D4_Out; //pinSolderProt (реле)
    D4_Low;
    D7_Out; //pinHotAirProt (реле)
    D7_Low;
    D8_Out; //pinHotAir (нагрев)
    D8_Low;
    D12_Out; //pinBuzzer
    D12_Low;
    ButtonsSetup();
 
#ifdef NEED_GRAPH
    Serial.begin(9600);
#endif
 
    lcd.init();
    lcd.backlight();
    lcd.createChar(0, degree);
    lcd.createChar(1, arrow);
    Splash();
    MemRead();
    
    delay_ms(2000);
    
    initDisplay();
    attachInterrupt(1, ZC, FALLING);
}
 
void loop() {
    ScanButtons();
 
    /* Отключение таймера для фена, обратный отсчет с 1 минутой */
    static uint32_t prevHAmillis = millis();
    if (hotair_state == st_pause) {
        if (millis() - prevHAmillis > 60000) {
            prevHAmillis = millis();
            if (HA_countdown > 1) {
                HA_countdown--;
                if (HA_countdown == 1) {
                    Beep(100);    //Звуковой сигнал, если осталось 1 минута
                }
            } else {
                hotair_state = st_stop;
                Beep(200);
            }
        }
    } else {
        prevHAmillis = millis();
    }
 
    /* Таймер отключения для паяльника, обратный отсчет с 1 минутой */
    static uint32_t prevSmillis = millis();
    if (need_S_countdown) {
        if (millis() - prevSmillis > 60000) {
            if (S_countdown > 1) {
                S_countdown--;
                if (S_countdown == 1) {
                    Beep(100);    //Звуковой сигнал, если осталось 1 минута
                }
            } else {
                MemSolder();
                need_S_countdown = false;
                SolderON = false;
                Beep(200);
            }
            prevSmillis = millis();
        }
    } else {
        prevSmillis = millis();
    }
 
    /* Обновить ЖК-дисплей с интервалом 100 мс */
    static uint32_t prevDisplayMillis = millis();
    static boolean blink_state = true;
    if (millis() - prevDisplayMillis > 100) {
        scr_blink = ! scr_blink;
        prevDisplayMillis = millis();
        DisplayUpdate(blink_state);
        blink_state = !blink_state;
 
        /* Отправка данных в последовательный порт */
#ifdef NEED_GRAPH
        if (SolderON || hotair_state == st_work) {
            Graph_count++;
            //Serial.print(Graph_count);
            //Serial.print(";");
        } else {
            Graph_count = 0;
        }
        if (SolderON && hotair_state != st_work) {
            Serial.println(GetSolderT);
        } else if (!SolderON && hotair_state == st_work) {
            Serial.println(GetHotAirT);
        }
#endif
    }
 
    WorkWithHotAir();
    WorkWithSolder();
}
 
/* Рабочая процедура HotAir */
void WorkWithHotAir() {
 
    /* Прочитать значение термопары */
#ifdef LM358
    GetHotAirT = getOversampled_HA();
#else
    GetHotAirT = getOversampled_HA() >> 1;
#endif
 
    switch (hotair_state) {
    case st_stop: {
        HotAirOff();
        Cooling();
        break;
    }
 
    case st_work: {
        /* Установите крутящий момент охладителя (конвертировать от 30-100% до 80-255 ШИМ) */
        analogWrite(pinFanPwm, map(SetHotAirRPM, min_rpm, max_rpm, 80, 255));
 
        ha_error = HADoProtect();
 
        /* Включить реле защиты */
        if (ha_error == 0) D7_High;
 
        /* Регулятор температуры PI */
        HA_PI();
 
        /* Если температура была стабильной в 100 раз (+/- 2 градуса), тогда сигнализируем об этом
 */
        int delta = ABS(SetHotAirT, GetHotAirT);
        static byte HAgood;
        if (!HA_temp_stable) {
            if (delta < 3) {
                HAgood++;
                if (HAgood == 100) {
                    Beep(50);
                    delay_ms(200);
                    Beep(50);
                    HAgood = 0;
                    HA_temp_stable = true;
                }
            } else {
                HAgood = 0;
            }
        } else {
            if (delta > 5) {
                HA_temp_stable = false;
            }
        }
        break;
    }
 
    case st_pause: {
        HAPower = 0.0;
        HA_temp_stable = false;
        CalctImpulseControl();
        Cooling();
        ha_error = HADoProtect();
        break;
    }
 
    case st_protection: {
        need_Cooling = true;
        break;
    }
 
    } //switch (state)
}
 
/* Рабочая процедура Solder */
void WorkWithSolder() {
 
    /* Прочитать значение терморезистора */
    //GetSolderT = A6_Read >> 1;
#ifdef LM358
    GetSolderT = getOversampled_S();
#else
    GetSolderT = getOversampled_S() >> 1;
#endif
 
    if ( SolderON ) {
        s_error = SDoProtect();
 
        /* Включить реле защиты */
        if (s_error == 0) D4_High;
 
        /* регулятор температуры P */
        S_P();
 
        /* Если их температура была стабильной для 200-кратного цикла (+/- 2 градуса), тогда сигнализируем об этом */
        uint16_t delta = ABS(SetSolderT, GetSolderT);
        static byte Sgood;
        if (!S_temp_stable) {
            if (delta < 3) {
                Sgood++;
                if (Sgood == 200) {
                    Beep(50);
                    delay_ms(200);
                    Beep(50);
                    //Sgood = 0;
                    S_temp_stable = true;
                }
            } else {
                Sgood = 0;
            }
        } else {
            if (delta > 5) {
                S_temp_stable = false;
            }
        }
    } else {
        SolderOff();
        need_S_countdown = false;
        S_countdown = 1;
    }
}
 
/***************************************** КОНЕЦ ОСНОВНЫХ ПРОЦЕДУР ******************************************/
 
 
 
/****************************************** ЗАЩИТА И ВКЛ-ВЫКЛ ********************************************/
 
/* Защита HotAir */
byte HADoProtect() {
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Crytical protection: высокое значение перегрева или термопары недействительно или разрыв провода */
    if (GetHotAirT > max_hotair_temp + 20) {
        HAProtectionOut();
        return 1;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Crytical protection: thermocouple value is not valid or wiresCrytical protection: значение термопары недействительно или короткое замыкание на провода short circuit */
    if (GetHotAirT < 10) {
        HAProtectionOut();
        return 2;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
#ifdef HA_ADV_PROT_ON
 
    /* Защита от перегрева*/
    if (ha_f1 && GetHotAirT > SetHotAirT + 20) {
        ha_f1 = false;
        HAProtectionOut();
        return 3;
    }
 
    if (ha_f2 && GetHotAirT < SetHotAirT + 15) {
        ha_f1 = true;
        ha_f2 = false;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Отклонение от температуры защиты +/- 10 градусов */
    //if (HA_temp_stable) {
    //ha_f3 = true;
    //}
//
    //if (ha_f3) {
    //if (GetHotAirT > SetHotAirT + 10) {
    //HAProtectionOut();
    //return 4;
    //}
//
    //if (GetHotAirT < SetHotAirT - 10) {
    //HAProtectionOut();
    //return 5;
    //}
    //}
 
    /* ------------------------------------------------------------------------------------------------------- */
 
 
 
    /* Расширенная защита: температура падает / не изменяется, а мощность> 0 */
    /* & */
    /* Усовершенствованная защита: повышение температуры и мощность <0 */
    static byte t_cnt = 0;
    static byte t_cnt2 = 0;
    static boolean ha_ctrl = true;
    if (!HA_temp_stable) {
        static uint16_t prev_t;
        if (ha_ctrl) {
            prev_t = GetHotAirT;
            ha_ctrl = false;
            prevHAcontrol = millis();
        }
 
        if (!ha_ctrl && millis() - prevHAcontrol > 1000) {
            ha_ctrl = true;
 
            if (HAPower > 0.0) {
 
                /* температура падает или не изменяется */
                if (prev_t >= GetHotAirT && GetHotAirT < SetHotAirT) {
                    t_cnt++;
                    if (t_cnt == 7) {
                        HAProtectionOut();
                        t_cnt = 0;
                        return 6;
                    }
                } else t_cnt = 0;
 
            } else { //HAPower == 0.0
 
                /* повышение температуры */
                if (prev_t < GetHotAirT && GetHotAirT > SetHotAirT) {
                    t_cnt2++;
                    if (t_cnt2 == 7) {
                        HAProtectionOut();
                        t_cnt2 = 0;
                        return 7;
                    }
                } else t_cnt2 = 0;
            }
        }
    } else {
        prevHAcontrol = millis();
        t_cnt = 0;
        t_cnt2 = 0;
        ha_ctrl = true;
    }
 
#endif
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* если все в порядке */
    return 0;
}
 
/* Защита паяльника */
byte SDoProtect() {
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Критическая защита: высокое значение перегрева или терморезистора недействительно или разрыв провода */
    if (GetSolderT > max_solder_temp + 20) {
        SProtectionOut();
        return 1;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Критическая защита: значение терморезистора недействительно или короткое замыкание на провода */
    if (GetSolderT < 10) {
        SProtectionOut();
        return 2;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
#ifdef S_ADV_PROT_ON
 
    /* Защита от перегрева */
    if (s_f1 && GetSolderT > SetSolderT + 20) {
        s_f1 = false;
        SProtectionOut();
        return 3;
    }
 
    if (s_f2 && GetSolderT < SetSolderT + 15) {
        s_f1 = true;
        s_f2 = false;
    }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* Отклонение от температуры защиты +/- 10 градусов */
    //if (S_temp_stable) {
    //s_f3 = true;
    //}
//
    //if (s_f3) {
    //if (GetSolderT > SetSolderT + 10) {
    //SProtectionOut();
    //return 4;
    //}
    //Not required for soldering iron
    //if (GetSolderT < SetSolderT - 10) {
    //SProtectionOut();
    //return 5;
    //}
    // }
 
    /* ------------------------------------------------------------------------------------------------------- */
 
 
 
    /* Расширенная защита: температура падает / не изменяется, а мощность> 0 */
    /* & */
    /* Усовершенствованная защита: повышение температуры и мощность <0 */
    static byte t_cnt = 0;
    static byte t_cnt2 = 0;
    static boolean s_ctrl = true;
    if (!S_temp_stable) {
        static uint16_t prev_t = 0;
        if (s_ctrl) {
            prev_t = GetSolderT;
            s_ctrl = false;
            prevScontrol = millis();
        }
 
        if (!s_ctrl && millis() - prevScontrol > 1000) {
            s_ctrl = true;
 
            if (SPower > 0) {
                /* температура падает или не изменяется */
                if (prev_t >= GetSolderT && GetSolderT < SetSolderT) {
                    t_cnt++;
                    if (t_cnt == 10) {
                        SProtectionOut();
                        t_cnt = 0;
                        return 6;
                    }
                } else t_cnt = 0;
            } else { //SPower == 0
                /* повышение температуры */
                if (prev_t < GetSolderT && GetSolderT > SetSolderT) {
                    t_cnt2++;
                    if (t_cnt2 == 10) {
                        SProtectionOut();
                        t_cnt2 = 0;
                        return 7;
                    }
                } else t_cnt2 = 0;
            }
        }
    } else {
        prevScontrol = millis();
        t_cnt = 0;
        t_cnt2 = 0;
        s_ctrl = true;
    }
 
#endif
 
    /* ------------------------------------------------------------------------------------------------------- */
 
    /* все в порядке */
    return 0;
}
 
/* Solder full off */
void SolderOff() {
    analogWrite(pinSolderPwm, 0);
    D5_Low;
    D4_Low;
    resetSolderStablePoint();
    if (S_prot_beep) {
        S_prot_beep = false;
        Beep(1000);
        MemSolder();
    }
}
 
/* HotAir full off */
void HotAirOff() {
    HAPower = 0.0;
    D8_Low;
    delay_ms(10);
    D7_Low;
    hotair_state = st_stop;
    resetHotAirStablePoint();
    if (HA_prot_beep) {
        HA_prot_beep = false;
        Beep(1000);
        MemHotAir();
    }
}
 
/* Охлаждение нагревателя до температуры ниже 50 градусов */
void Cooling() {
    if (GetHotAirT >= 50 && need_Cooling) {
        analogWrite(pinFanPwm, 255);
    } else {
        analogWrite(pinFanPwm, 0);
        need_Cooling = false;
    }
}
 
/* внутренняя процедура */
void set_ha_f() {
    boolean a = SetHotAirT >= GetHotAirT;
    ha_f1 = a;
    ha_f2 = !a;
}
 
/* внутренняя процедура */
void set_s_f() {
    boolean a = SetSolderT >= GetSolderT;
    s_f1 = a;
    s_f2 = !a;
}
 
/* Защита паяльника */
void SProtectionOut() {
    SolderProtect = true;
    S_prot_beep = true;
    SolderON = false;
    SolderOff();
}
 
/* Защита фена */
void HAProtectionOut() {
    analogWrite(pinFanPwm, 255);
    HA_prot_beep = true;
    HotAirOff();
    hotair_state = st_protection;
}
 
/****************************************** КОНЕЦ ЗАЩИТЫ И ВКЛ-ВЫКЛ *************************************/
 
 
 
/************************************************ ДРУГИЕ РЕЖИМЫ *******************************************/
 
/* Чтение целочисленного значения */
int EEPROM_int_read(int addr) {
    byte raw[2];
    for (byte i = 0; i < 2; i++) {
        raw[i] = EEPROM.read(addr + i);
    }
    int &num = (int&)raw;
    return num;
}
 
/* Запись целочисленного значения */
void EEPROM_int_write(int addr, int num) {
    byte raw[2];
    (int&)raw = num;
    for (byte i = 0; i < 2; i++) {
        EEPROM.write(addr + i, raw[i]);
    }
}
 
/* Чтение последних параметров из памяти */
void MemRead() {
    SetSolderT = EEPROM_int_read(0);
    if (SetSolderT < min_solder_temp || SetSolderT > max_solder_temp)  {
        SetSolderT = default_temp;
        last_SolderT = SetSolderT;
    }
    SetHotAirT = EEPROM_int_read(4);
    if (SetHotAirT < min_hotair_temp || SetHotAirT > max_hotair_temp) {
        SetHotAirT = default_temp;
        last_HotAirT = SetHotAirT;
    }
    SetHotAirRPM = EEPROM_int_read(8);
    if (SetHotAirRPM < min_rpm || SetHotAirRPM > max_rpm) {
        SetHotAirRPM = default_rpm;
        last_RPM = SetHotAirRPM;
    }
}
 
/* Записывать последнюю использованную температуру паяльника в память */
void MemSolder() {
    if (last_SolderT != SetSolderT) {
        EEPROM_int_write(0, SetSolderT);
        last_SolderT = SetSolderT;
    }
}
 
/* Записывать последнюю использованную температуру фена и обороты куллера в память */
void MemHotAir() {
    if (last_HotAirT != SetHotAirT) {
        EEPROM_int_write(4, SetHotAirT);
        last_HotAirT = SetHotAirT;
    }
    if (last_RPM != SetHotAirRPM) {
        EEPROM_int_write(8, SetHotAirRPM);
        last_RPM = SetHotAirRPM;
    }
}
 
/* Звуковая процедура */
void Beep(uint16_t duration) {
#ifdef PASSIVE_BUZZER
    tone(pinBuzzer, 1000, duration);
#else
    D12_High;
    delay_ms(duration);
    D12_Low;
#endif
}
 
/* Функция передискретизации HotAir */
//uint16_t getOversampled_HA() {
//uint32_t tmp = 0;
//for (byte z = 0; z < 64; z++) {
//tmp +=  A7_Read;
//}
//return tmp >> 6;
//}
 
uint16_t getOversampled_HA() {
    uint32_t tmp = 0;
    for (byte z = 0; z < 128; z++) {
        tmp +=  A7_Read;
    }
    return tmp >> 7;
}
 
/* Функция передискретизации паяльника */
uint16_t getOversampled_S() {
    uint32_t tmp = 0;
    for (byte z = 0; z < 64; z++) {
        tmp +=  A6_Read;
    }
    return tmp >> 6;
}
 
/* Получить функцию абсолютной разности */
uint16_t ABS(uint16_t a, uint16_t b) {
    if (a > b) {
        return (a - b);
    }
    return (b - a);
}
 
/****************************************** КОНЕЦ ДРУГИХ РЕЖИМОВ ******************************************/
 
 
 
/*************************************** АВТОМАТИЧЕСКОЕ УПРАВЛЕНИЕ СИМИСТОРОМ ****************************************/
 
/* Zero cross INT1 */
void ZC() {
    StartTimer1(HeaterOn, ots);
    RestartTimer1();
}
 
/* Triac open impulse */
void HeaterOn() {
    StopTimer1();
    if (HAPower > 0.0) {
        D8_High;
        delay_us(30);
    }
    D8_Low;
}
 
/* Вычислить задержку триггера */
void CalctImpulseControl() {
    ots = (uint16_t)(acos(HAPower / 50.0 - 1.0 ) * 9900.0 / pi);
}
 
/* Регулятор HotAir PI */
void HA_PI() {
    int err = SetHotAirT - GetHotAirT;
    float tmp_power = ((Kp * (float)err) + (Ki * (float)integral));
    float max_power = map((float)SetHotAirT, min_hotair_temp, max_hotair_temp, 10.0, 60.0);
    if (tmp_power < max_power && tmp_power > 0.0) {
        integral += err;
    }
    HAPower = constrain(tmp_power, 0.0, max_power);
    CalctImpulseControl();
}
 
 
#ifdef SOLDER_SOFT_START
/* Регулятор Solder P с плавным пуском */
void S_P() {
    int TempPower = sKp * (SetSolderT - GetSolderT + 1);
    byte maxPower = 255;
    if (GetSolderT < 100) {
        maxPower = 100;
    }
    SPower = constrain(TempPower, 0, maxPower);
    analogWrite(pinSolderPwm, SPower);
}
#else
/* Регулятор Solder P */
void S_P() {
    int TempPower = sKp * (SetSolderT - GetSolderT + 1);
    SPower = constrain(TempPower, 0, 255);
    analogWrite(pinSolderPwm, SPower);
}
#endif
 
/************************************* END АВТОМАТИЧЕСКОЕ УПРАВЛЕНИЕ СИМИСТОРОМ ***********************************/
 
 
 
/****************************************КОНТРОЛЬ ИНТЕРФЕЙСА И ТАЙМЕРЫ **************************************/
 
/* сканирование кнопок */
void ScanButtons() {
 
    /*  переключатель подсставки фена */
    if (swHotAir.update() && hotair_state != st_stop) {
        if (swHotAir.fell()) {
            hotair_state = st_pause;
            need_Cooling = true;
            resetHotAirCountown();
        } else {
            if (hotair_state == st_pause) {
                hotair_state = st_work;
                integral = 0;
                set_ha_f() ;
            }
        }
        Beep(50);
    } else if (hotair_state == st_work && D10_Read == LOW) {
        D7_High;
        hotair_state = st_pause;
        need_Cooling = true;
        resetHotAirCountown();
    }
 
#ifdef SOLDER_TIMER_ON_STAND
 
    /* Выключатель подставки паяльника - только на подставке */
    if (swSolder.update()) {
        if (swSolder.fell() && SolderON) {
            if (!need_S_countdown) {
                Activate_S_countdown();
                Beep(50);
            }
        }
 
        if (swSolder.rose())  {
            if (need_S_countdown) {
                need_S_countdown = false;
                Beep(50);
            }
        }
    }
 
#else
 
    /* Выключатель подставки для паяльника простой проверки активности пользователя */
    if (swSolder.update() && SolderON) {
        resetSolderCountdown();
        Beep(50);
    }
 
#endif // SOLDER_TIMER_ON_STAND
 
    /* Кнопка паяльника «включено-выключено» */
    if (SolderOnButton.update()) {
        if (SolderOnButton.read()) {
            SONbuttonState = false;
        } else {
            if (!SolderON) {
                SolderON = true;
                SolderProtect = false;
 
#ifdef SOLDER_TIMER_ON_STAND
                if (!D9_Read) Activate_S_countdown();
#else
                Activate_S_countdown();
#endif
 
                Graph_count = 0;
                selected_Mode = modeSolder;
                resetSolderStablePoint();
                set_s_f();
            } else {
                resetSolderCountdown();
            }
            SONbuttonState = true;
            SONbuttonPressTime = millis();
            Beep(50);
        }
    }
 
    if  (SONbuttonState) {
        if ( millis() - SONbuttonPressTime >= 1000 ) { //удержание кнопки
            SONbuttonPressTime = millis();
            if (SolderON) {
                SolderON = false;
                resetSolderStablePoint();
                MemSolder();
                Beep(200);
            }
        }
    }
 
    /* HotAir "on-off" button */
    if (HotAirOnButton.update()) {
        if (HotAirOnButton.read()) {
            HAONbuttonState = false;
        } else {
            if (hotair_state != st_work) {
                hotair_state = st_work;
                Graph_count = 0;
                selected_Mode = modeHotAir;
                need_Cooling = true;
                resetHotAirStablePoint();
                set_ha_f();
                integral = 0;
            } else {
                resetHotAirCountown();
            }
            HAONbuttonState = true;
            HAONbuttonPressTime = millis();
            Beep(50);
        }
    }
 
    if  (HAONbuttonState) {
        if ( millis() - HAONbuttonPressTime >= 1000 ) { //long press
            HAONbuttonPressTime = millis();
            resetHotAirStablePoint();
            if (hotair_state == st_work || hotair_state == st_pause) {
                hotair_state = st_stop;
                need_Cooling = true;
                MemHotAir();
                Beep(200);
            }
        }
    }
 
#ifdef ENCODER    
    /* Короткое нажатие кнопки Encoder */
    if (Enc_Button.update()) {
        if (Enc_Button.fell()) {
            Beep(50);
            Enc_ButtonPressTime = millis();
            Enc_ButtonState = true;
        } else {
            if (Enc_ButtonState) {
                (selected_Mode < modeFanPWM) ? (selected_Mode++) : (selected_Mode = modeSolder);
            }
            Enc_ButtonState = false;
        }
    }
#else
    /* Кнопка выбора */
    if (SelButton.update()) {
        if (SelButton.fell()) {
            Beep(50);
            (selected_Mode < modeFanPWM) ? (selected_Mode++) : (selected_Mode = modeSolder);
        }
    }
 
    /* Кнопка ВВЕРХ */
    static boolean short_press_flag = false;
 
    if (UpButton.update()) {
        if (UpButton.rose()) {
            UPbuttonState = false;
            short_press_flag = false;
        } else {
            switch (selected_Mode) {
            case modeSolder:
                if (SetSolderT < max_solder_temp) {
                    SetSolderT += 5;
                }
                set_s_f();
                resetSolderStablePoint();
                resetSolderCountdown();
                break;
            case modeHotAir:
                if (SetHotAirT < max_hotair_temp) {
                    SetHotAirT += 5;
                }
                break;
            case modeFanPWM:
                if (SetHotAirRPM < max_rpm) {
                    SetHotAirRPM += 5;
                }
                break;
            }
 
            if (selected_Mode > 1) {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
                integral = 0;
            }
 
            UPbuttonState = true;
            short_press_flag = true;
            UPbuttonPressTime = millis();
            Beep(50);
        }
    }
 
    /* Кнопка вверх (длительное нажатие) */
    if  (UPbuttonState) {
        if ( millis() - UPbuttonPressTime >= 500 ) {
            UPbuttonPressTime = millis();
            byte step = 10;
            if (short_press_flag) {
                step = 5;
                short_press_flag = false;
            }
            switch (selected_Mode) {
            case modeSolder:
                SetSolderT += step;
                if (SetSolderT > max_solder_temp) SetSolderT = max_solder_temp;
                resetSolderStablePoint();
                set_s_f();                
                resetSolderCountdown();
                break;
            case modeHotAir:
                SetHotAirT += step;
                if (SetHotAirT > max_hotair_temp) SetHotAirT = max_hotair_temp;
                break;
            case modeFanPWM:
                SetHotAirRPM += step;
                if (SetHotAirRPM > max_rpm) SetHotAirRPM = max_rpm;
                break;
            }
            if (selected_Mode > 1) {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
                integral = 0;
            }
        }
    }
 
    /* Кнопка Вниз */
    if (DwnButton.update()) {
        if (DwnButton.read()) {
            DWNbuttonState = false;
            short_press_flag = false;
        } else {
            switch (selected_Mode) {
            case modeSolder:
                if (SetSolderT > min_solder_temp) {
                    SetSolderT -= 5;
                }                
                set_s_f();
                resetSolderStablePoint();
                resetSolderCountdown();
                break;
            case modeHotAir:
                if (SetHotAirT > min_hotair_temp) {
                    SetHotAirT -= 5;
                }
                break;
            case modeFanPWM:
                if (SetHotAirRPM > min_rpm) {
                    SetHotAirRPM -= 5;
                }
                break;
            }
 
            if (selected_Mode > 1) {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
                //integral = 0;
            }
 
            DWNbuttonState = true;
            short_press_flag = true;
            DWNbuttonPressTime = millis();
            Beep(50);
        }
    }
 
    /* Кнопка «Вниз» (длительное нажатие) */
    if  (DWNbuttonState) {
        if ( millis() - DWNbuttonPressTime >= 500 ) {
            DWNbuttonPressTime = millis();
 
            byte step = 10;
            if (short_press_flag) {
                step = 5;
                short_press_flag = false;
            }
 
            switch (selected_Mode) {
            case modeSolder:
                SetSolderT -= step;
                if (SetSolderT < min_solder_temp) SetSolderT = min_solder_temp;
                set_s_f();
                resetSolderStablePoint();
                resetSolderCountdown();
                break;
            case modeHotAir:
                SetHotAirT -= step;
                if (SetHotAirT < min_hotair_temp) SetHotAirT = min_hotair_temp;
                break;
            case modeFanPWM:
                SetHotAirRPM -= step;
                if (SetHotAirRPM < min_rpm) SetHotAirRPM = min_rpm;
                break;
            }
 
            if (selected_Mode > 1) {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
                //integral = 0;
            }
        }
    }
#endif
}
 
#ifdef ENCODER 
/* Encoder interrrupt */
ISR(PCINT1_vect) {
    static byte old_n=PINC & B1100; // ????? B00001100 ??? ? ?????? ?????? ?????? 2 ????
    byte new_n = PINC & B1100;
 
    if (old_n == B0100 && new_n == B1100 || old_n == B1000 && new_n == B0000) { //CW turn
            
            switch (selected_Mode)
      {
            case modeSolder:
                if (SetSolderT < max_solder_temp)
              { SetSolderT += 5; }
                resetSolderStablePoint();
                set_s_f();
                resetSolderCountdown();
                break;
            case modeHotAir:
                if (SetHotAirT < max_hotair_temp)
        { SetHotAirT += 5; }
                break;
            case modeFanPWM:
                if (SetHotAirRPM < max_rpm) 
        { SetHotAirRPM += 5; }
                break;
            }
 
            if (selected_Mode > 1) 
      {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
            }
            Beep(50);
    }
    
    if (old_n == B1000 && new_n == B1100 || old_n == B0100 && new_n == B0000) { //CCW turn
            
            switch (selected_Mode) 
      {
            case modeSolder:
                if (SetSolderT > min_solder_temp)
              { SetSolderT -= 5; }
                resetSolderStablePoint();
                set_s_f();
                resetSolderCountdown();
                break;
            case modeHotAir:
                if (SetHotAirT > min_hotair_temp) 
        { SetHotAirT -= 5; }
                break;
            case modeFanPWM:
                if (SetHotAirRPM > min_rpm) 
        { SetHotAirRPM -= 5; }
                break;
            }
 
            if (selected_Mode > 1) 
      {
                set_ha_f();
                resetHotAirStablePoint();
                resetHotAirCountown();
            }
            Beep(50);
    }
    old_n = new_n;
}
#endif
 
/* Инициализация кнопок */
void ButtonsSetup() {
    pinMode(sw_HA, INPUT_PULLUP);
    swHotAir.attach(sw_HA);
    swHotAir.interval(50);
 
    pinMode(sw_S, INPUT_PULLUP);
    swSolder.attach(sw_S);
    swSolder.interval(50);
 
    pinMode(bt_SON, INPUT_PULLUP);
    SolderOnButton.attach(bt_SON);
    SolderOnButton.interval(5);
 
    pinMode(bt_HAON, INPUT_PULLUP);
    HotAirOnButton.attach(bt_HAON);
    HotAirOnButton.interval(5);
 
#ifdef ENCODER
    pinMode(e_button, INPUT_PULLUP);//enc button 
    Enc_Button.attach(e_button);
    Enc_Button.interval(5);
    
    pinMode(e_dir, INPUT_PULLUP);   //pin A2 enc_a
    
    pinMode(e_step, INPUT_PULLUP);  //pin A3 enc_b
 
    PCICR=1<<PCIE1;     // enable interrrupt for group 1
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.   
#else
    pinMode(bt_Sel, INPUT_PULLUP);
    SelButton.attach(bt_Sel);
    SelButton.interval(10);
 
    pinMode(bt_Up, INPUT_PULLUP);
    UpButton.attach(bt_Up);
    UpButton.interval(5);
 
    pinMode(bt_Dwn, INPUT_PULLUP);
    DwnButton.attach(bt_Dwn);
    DwnButton.interval(5);
#endif
}
 
/* внутренняя процедура */
void resetHotAirStablePoint() {
    HA_temp_stable = false;
}
 
/* внутренняя процедура */
void resetSolderStablePoint() {
    S_temp_stable = false;
}
 
/* Сбросить обратный отсчет HotAir */
void resetHotAirCountown() {
    HA_countdown = HA_sleeptime;
}
 
/* Сброс обратного отсчета паялька */
void resetSolderCountdown() {
    S_countdown = S_sleeptime;
}
 
/* Активировать процедуру обратного отсчета паяльника */
void Activate_S_countdown() {
    need_S_countdown = true;
    S_countdown = S_sleeptime;
}
 
/**************************************** КОНТРОЛЬ ИНТЕРФЕЙСА И ТАЙМЕРЫ **************************************/
 
 
/************************************************* DISPLAY *************************************************/
 
/* "Hello" screen */
void Splash() {
    lcd.clear();
#ifdef LCD_1602
    lcd.setCursor(0, 0);
    lcd.print(F("Soldering Station"));
    lcd.setCursor(6, 1);
#else
    lcd.setCursor(2, 1);
    lcd.print(F("Soldering Station"));
    lcd.setCursor(7, 2);
#endif
    lcd.print(F("v "));
    lcd.print(FIRMWARE_VERSION);
}
 
/*Настройка ЖК-дисплея */
void initDisplay() {
    lcd.clear();
#ifdef LCD_1602
 
#else
    lcd.setCursor(11, 0);
    lcd.print(F("Set"));
 
    lcd.setCursor(16, 0);
    lcd.print(F("Act"));
 
    lcd.setCursor(0, 1);
    lcd.print(F("Solder"));
 
    lcd.setCursor(0, 2);
    lcd.print(F("HotAir"));
 
    lcd.setCursor(0, 3);
    lcd.print(F("FanRPM"));
#endif
}
 
/* Обновить ЖК-дисплей */
void DisplayUpdate(boolean blink_state) {
 
#ifdef LCD_1602
 
    lcd.setCursor(0, 0);
    lcd.print(bspace);
    lcd.setCursor(0, 0);
 
    if (SolderProtect) {
        lcd.print(F("!"));
        lcd.print(s_error);
    } else if (need_S_countdown) {
        if (blink_state) lcd.print(S_countdown);
    } else if (SolderON) {
        (S_temp_stable) ? (lcd.print(F(" *"))) : (lcd.print(F(" :")));
    }
 
    lcd.setCursor(3, 0);
    lcd.print(bspace);
    lcd.setCursor(3, 0);
    lcd.print(SetSolderT);
    lcd.write((byte)0);
 
    byte pos;
    uint16_t s_view_t;
 
    if (GetSolderT > 505) {
        lcd.setCursor(8, 0);
        lcd.print(F("---"));
    } else {
        uint16_t s_view_t;
#ifdef SOLDER_DIGIT_JUMPS_REMOVE
        (S_temp_stable)? (s_view_t = SetSolderT) : (s_view_t = GetSolderT);
#else
        s_view_t = GetSolderT;
#endif
        pos = GetPos(s_view_t);
        lcd.setCursor(8, 0);
        lcd.print(bspace);
        lcd.setCursor(8+pos, 0);
        lcd.print(s_view_t);
    }
    lcd.write((byte)0);
 
    lcd.setCursor(0, 1);
    lcd.print(bspace);
    lcd.setCursor(0, 1);
 
    switch (hotair_state) {
    case st_stop: {
        break;
    }
 
    case st_work: {
        if (HA_temp_stable) {
            lcd.print(F(" *"));
        } else {
            lcd.print(F(" :"));
        }
        break;
    }
 
    case st_pause: {
        if (blink_state) lcd.print(HA_countdown);
        break;
    }
 
    case st_protection: {
        lcd.print(F("!"));
        lcd.print(ha_error);
        break;
    }
 
    }
 
    pos = GetPos(SetHotAirT);
    lcd.setCursor(3, 1);
    lcd.print(bspace);
    lcd.setCursor(3+pos, 1);
    lcd.print(SetHotAirT);
    lcd.write((byte)0);
 
    if (GetHotAirT > 505) {
        lcd.setCursor(8, 1);
        lcd.print(F("---"));
    } else {
        uint16_t ha_view_t;
#ifdef HOTAIR_DIGIT_JUMPS_REMOVE
        (HA_temp_stable) ? (ha_view_t = SetHotAirT) : (ha_view_t = GetHotAirT);
#else
        ha_view_t = GetHotAirT;
#endif
        pos = GetPos(ha_view_t);
        lcd.setCursor(8, 1);
        lcd.print(bspace);
        lcd.setCursor(8+pos, 1);
        lcd.print(ha_view_t);
    }
    lcd.write((byte)0);
 
    lcd.setCursor(13, 1);
    lcd.print(bspace);
    lcd.setCursor(13, 1);
 
    pos = GetPos(SetHotAirRPM);
    if (need_Cooling && hotair_state != st_work) {
        lcd.print(F("100%"));
    } else {
        lcd.setCursor(13+pos, 1);
        lcd.print(SetHotAirRPM);
        lcd.print(F("%"));
    }
 
    lcd.setCursor(2, 0);
    lcd.print(F(" "));
    lcd.setCursor(2, 1);
    lcd.print(F(" "));
    lcd.setCursor(12, 1);
    lcd.print(F(" "));
 
    switch (selected_Mode) {
    case modeSolder: {
        lcd.setCursor(2, 0);
        break;
    }
 
    case modeHotAir: {
        lcd.setCursor(2, 1);
        break;
    }
 
    case modeFanPWM: {
        lcd.setCursor(12, 1);
        break;
    }
    }
    lcd.write((byte)1);
 
#else //LCD2004
 
    lcd.setCursor(6, 1);
    lcd.print(bspace);
    lcd.setCursor(7, 1);
 
    if (SolderProtect) {
        lcd.print(F("!"));
        lcd.print(s_error);
    } else if (need_S_countdown) {
        if (blink_state) lcd.print(S_countdown);
    } else if (SolderON) {
        (S_temp_stable) ? (lcd.print(F(" *"))) : (lcd.print(F(" :")));
    }
 
    lcd.setCursor(11, 1);
    lcd.print(bspace);
    lcd.setCursor(11, 1);
    lcd.print(SetSolderT);
    lcd.write((byte)0);
 
    byte pos;
    uint16_t s_view_t;
 
    if (GetSolderT > 505) {
        lcd.setCursor(16, 1);
        lcd.print(F("---"));
    } else {
        uint16_t s_view_t;
#ifdef SOLDER_DIGIT_JUMPS_REMOVE
        (S_temp_stable) ? (s_view_t = SetSolderT) : (s_view_t = GetSolderT);
#else
        s_view_t = GetSolderT;
#endif
        pos = GetPos(s_view_t);
        lcd.setCursor(16, 1);
        lcd.print(bspace);
        lcd.setCursor(16+pos, 1);
        lcd.print(s_view_t);
    }
    lcd.write((byte)0);
 
    lcd.setCursor(6, 2);
    lcd.print(bspace);
    lcd.setCursor(7, 2);
 
    switch (hotair_state) {
    case st_stop: {
        break;
    }
 
    case st_work: {
        if (HA_temp_stable) {
            lcd.print(F(" *"));
        } else {
            lcd.print(F(" :"));
        }
        break;
    }
 
    case st_pause: {
        if (blink_state) lcd.print(HA_countdown);
        break;
    }
 
    case st_protection: {
        lcd.print(F("!"));
        lcd.print(ha_error);
        break;
    }
 
    }
 
    pos = GetPos(SetHotAirT);
    lcd.setCursor(11, 2);
    lcd.print(bspace);
    lcd.setCursor(11+pos, 2);
    lcd.print(SetHotAirT);
    lcd.write((byte)0);
 
    if (GetHotAirT > 505) {
        lcd.setCursor(16, 2);
        lcd.print(F("---"));
    } else {
        uint16_t ha_view_t;
#ifdef HOTAIR_DIGIT_JUMPS_REMOVE
        (HA_temp_stable) ? (ha_view_t = SetHotAirT) : (ha_view_t = GetHotAirT);
#else
        ha_view_t = GetHotAirT;
#endif
        pos = GetPos(ha_view_t);
        lcd.setCursor(16, 2);
        lcd.print(bspace);
        lcd.setCursor(16+pos, 2);
        lcd.print(ha_view_t);
    }
    lcd.write((byte)0);
 
    lcd.setCursor(11, 3);
    lcd.print(bspace);
    lcd.setCursor(11, 3);
 
    pos = GetPos(SetHotAirRPM);
    if (need_Cooling && hotair_state != st_work) {
        lcd.print(F("100%"));
    } else {
        lcd.setCursor(11+pos, 3);
        lcd.print(SetHotAirRPM);
        lcd.print(F("%"));
    }
 
    for (byte z = 1; z < 4; z++) {
        lcd.setCursor(10, z);
        lcd.print(F(" "));
    }
 
    lcd.setCursor(10, selected_Mode);
    lcd.write((byte)1);
 
#endif
 
}
 
/* Get print position */
byte GetPos(uint16_t number) {
    if (number >= 100) {
        return 0;
    } else if (number < 10) {
        return 2;
    }
    return 1;
}
 
/*********************************************** END OF DISPLAY ********************************************/

улучши весь код
оптимизируй весь код
Выдай готовый код со всеми улучьшениями и оптимизациями
глубокий анализ всего кода

 
