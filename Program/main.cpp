#include "mbed.h"
#include <string.h>
#define gon_address 0x80
//#define sv_cycle 13   //ms
#define sv_cycle 25 //ms
#define buf_size 100
/*
#define s1_speed 50
#define s2_speed 33
#define s3_speed 20
#define s4_speed 50
#define s5_speed 33
#define s6_speed 20
#define s7_speed 10
#define s8_speed 10
*/
#define kata_speed 20
#define hiji_speed 33
#define teku_speed 20
#define kubi_speed 50

 
double map(double v, double sn, double sx, double dn, double dx)    /*value max min max min*/
{return ((v - sn) * (dx - dn) / (sx - sn) + dn);}
double constrain(double x,double min, double max){
    if(min > x)
        return min;
    else if (max < x)
        return max;
    else
        return x;
}


/*  class  */
Serial pc(USBTX, USBRX);
Serial s1(PA_15,PB_7);
Serial xb(PA_11,PA_12);
I2C gon(PB_4,PA_8);
DigitalOut myled(LED1);
PwmOut sv8(PC_8);   // 3/3
PwmOut sv7(PC_6);   // 3/1
PwmOut sv6(PC_9);   // 3/4
PwmOut sv5(PB_8);   // 4/3
PwmOut sv4(PB_9);   // 4/4
PwmOut sv2(PA_7);   // 1/1n
PwmOut sv3(PA_5);   // 2/1
PwmOut sv1(PC_7);   // 3/2
DigitalOut R1(PC_11);
DigitalOut R2(PD_2);
DigitalOut L1(PC_12);
DigitalOut L2(PC_10);
DigitalOut M0(PB_0);
#define Rkata sv1
#define Rhiji sv2
#define Rteku sv3
#define Lkata sv4
#define Lhiji sv5
#define Lteku sv6
#define Nyoko sv7
#define Ntate sv8

Ticker sb1;
Ticker sb2;
Ticker sb3;
Ticker sb4;
Ticker sb5;
Ticker sb6;
Ticker sb7;
Ticker sb8;
Timeout Mt;
int mtban=1;

 /* interruput */
double P_min[]={   0, 544, 544, 544, 544, 544, 544, 544, 600};
double P_max[]={2400,2200,2400,2400,2400,2400,2400,2400,2325};
double D_now[9];
double D_tar[]={   0,  90,  97,  00,  90,  80, 160,  80,  90};
double P_now[]={1000,1000,1000,1000,1000,1000,1000,1000,1000};
double P_tar[]={1000,1000,1000,1000,1000,1000,1000,1000,1000};

void D_set(int no,double deg) {D_tar[no]=deg;P_tar[no]=(double)map(deg ,0 ,180 ,P_min[no],P_max[no]);}
void D_set(int no,   int deg) {D_tar[no]=(double)deg;P_tar[no]=map((double)deg ,0 ,180 ,P_min[no],P_max[no]);}
void P_set(int no,double pw) {P_tar[no]=pw;D_tar[no]=map(pw ,P_min[no] ,P_max[no] ,0 ,180);}
void sb1_func();
void sb2_func();
void sb3_func();
void sb4_func();
void sb5_func();
void sb6_func();
void sb7_func();
void sb8_func();
/*
void sb1_funcP();
void sb2_funcP();
void sb3_funcP();
void sb4_funcP();
void sb5_funcP();
void sb6_funcP();
void sb7_funcP();
void sb8_funcP();
*/

/* モーターの前後 */
void R_for();
void R_rev();
void R_stp();
void R_brk();
void L_for();
void L_rev();
void L_stp();
void L_brk();
void M_for();
void M_rev();
void M_stp();
void M_brk();
void M_R_turn();
void M_L_turn();

/*servo motor set*/
void sv_set(int no, double deg);
void sv_setP(int no, double pw);
void sv_set_neutral();  //all deg90
void sv_set_pass();     //仮実装
void sv_set_debug();    //super debug
void sv_set_manual();   //manual set
void sv_set_chk();
void M_push();          //名刺出す
void M_pull();          //名刺止める
void kubi_set(int);
void kubi_shuffle();

/*  SerialRW & modechange  */
char mode='Q';
char oldmode='0';
char pc_wd[buf_size];
char s1_wd[buf_size];
char xb_wd[buf_size];
char md_wd[buf_size];
int pcc=0;
int s1c=0;
int xbc=0;
void pc_read();
void s1_read();
void xb_read();
void serial_buf_reset();
void serial_buf_clear();
void mode_change(char);
char mode_judge();
bool PR=false;
bool SS=false;
char gon_data[24];
void gon_trans(char red[8],char green[8],char blue[8]);
int gon_temp=0;
void gon_wink();
char gon_ue[8]={
    24,     //  0b00011000,
    60,     //  0b00111100,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    60,     //  0b00111100,
    60,     //  0b00111100,
    60      //  0b00111100  
};
char gon_ue2[8]={
    231,    //  0b11100111,
    195,    //  0b11000011,
    129,    //  0b10000001,
    0,      //  0b00000000,
    0,      //  0b00000000,
    195,    //  0b11000011,
    195,    //  0b11000011,
    195     //  0b11000011
};
char gon_sita[8]={
    60,     //  0b00111100,
    60,     //  0b00111100,
    60,     //  0b00111100,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60,     //  0b00111100,
    24      //  0b00011000
};
char gon_sita2[8]={
    ~60,    //  0b11000011,
    ~60,    //  0b11000011,
    ~60,    //  0b11000011,
    0,      //  0b00000000,
    0,      //  0b00000000,
    ~126,   //  0b10000001,
    ~60,    //  0b11000011,
    ~24     //  0b11100111
};
char gon_migi[8]={
    24,     //  0b00011000,
    28,     //  0b00011100,
    254,    //  0b11111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    254,    //  0b11111110,
    28,     //  0b00011100,
    24      //  0b00011000
};
char gon_hidari[8]={
    24,     //  0b00011000,
    56,     //  0b00111000,
    127,    //  0b01111111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    127,    //  0b01111111,
    56,     //  0b00111000,
    24      //  0b00011000
};
char gon_warai[8]={
    0,      //  0b00000000,
    66,     //  0b01000010,
    165,    //  0b10100101,
    0,      //  0b00000000,
    0,      //  0b00000000,
    66,     //  0b01000010,
    36,     //  0b00100100,
    24      //  0b00011000  
};  
char gon_me1[8]={
    60,     //  0b00111100,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60      //  0b00111100  
};
char gon_me2[8]={
    24,     //  0b00011000,
    126,    //  0b01111110,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    126,    //  0b01111110,
    24      //  0b00011000
};
char gon_warau11[8]={
    60,     //  0b00111100,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    231,    //  0b11100111,
    129,    //  0b10000001,
    0       //  0b00000000
};
char gon_warau12[8]={
    24,     //  0b00011000,
    126,    //  0b01111110,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    195,    //  0b11000011,
    129,    //  0b10000001,
    0       //  0b00000000
};
char gon_warau21[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    60,     //  0b00111100,
    126,    //  0b01111110,
    231,    //  0b11100111,
    195,    //  0b11000011,
    129,    //  0b10000001,
    0       //  0b00000000
};
char gon_warau22[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    24,     //  0b00011000,
    126,    //  0b01111110,
    66,     //  0b01000010,
    129,    //  0b10000001,
    0,      //  0b00000000,
    0       //  0b00000000
};
char gon_okoru11[8]={
    0,      //  0b00000000,
    129,    //  0b10000001,
    231,    //  0b11100111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60      //  0b00111100
};
char gon_okoru12[8]={
    0,      //  0b00000000,
    129,    //  0b10000001,
    195,    //  0b11000011,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    126,    //  0b01111110,
    24      //  0b00011000
};
char gon_okoru21[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    129,    //  0b10000001,
    231,    //  0b11100111,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    0       //  0b00000000
};
char gon_okoru22[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    129,    //  0b10000001,
    195,    //  0b11000011,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60,     //  0b00111100,
    0       //  0b00000000
};
char gon_mabataki01[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    126,    //  0b01111110,
    126,    //  0b01111110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60      //  0b00111100
};
char gon_mabataki02[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    56,     //  0b00111000,
    60,     //  0b00111100,
    255,    //  0b11111111,
    126,    //  0b01111110,
    126,    //  0b01111110,
    24      //  0b00011000
};
char gon_mabataki11[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    60      //  0b00111100
};
char gon_mabataki12[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    255,    //  0b11111111,
    126,    //  0b01111110,
    126,    //  0b01111110,
    24      //  0b00011000
};
char gon_mabataki21[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,    //  0b00000000,
    129,    //  0b10000001,
    195,    //  0b11000011,
    102,    //  0b01100110,
    60      //  0b00111100
};
char gon_mabataki22[8]={
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,      //  0b00000000,
    0,    //  0b00000000,
    129,     //  0b10000001,
    66,    //  0b01000010,
    24      //  0b00011000
};
char gon_love1[8]={
    0,      //  0b00000000,
    102,    //  0b01100110,
    255,    //  0b11111111,
    255,    //  0b11111111,
    126,    //  0b01111110,
    126,    //  0b01111110,
    60,     //  0b00111100,
    24      //  0b00011000
};
char gon_love2[8]={
    0,      //  0b00000000,
    108,    //  0b01101100,
    252,    //  0b11111100,
    252,    //  0b11111100,
    124,    //  0b01111100,
    124,    //  0b01111100,
    56,     //  0b00111000,
    16      //  0b00010000
};
char gon_magao[8]={
    0,      //  0b00000000,
    102,    //  0b01100110,
    102,    //  0b01100110,
    0,      //  0b00000000,
    0,      //  0b00000000,
    126,    //  0b01111110,
    126,    //  0b01111110,
    0,      //  0b00000000  
};
char gon_ouen[8]={
    0,      //  0b00000000,
    129,    //  0b10000001,
    66,     //  0b01000010,
    36,     //  0b00100100,
    36,     //  0b00100100,
    66,     //  0b01000010,
    129,    //  0b10000001,
    0,      //  0b00000000
};          
char gon_all[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
char gon_none[8]={0,0,0,0,0,0,0,0};
void gon_wink01();
void gon_wink02();
void gon_wink03();
void printmd_wd(){
        if(PR){
        for(int i = 0;i<buf_size;i++){
        pc.printf("%c",md_wd[i]);}
        pc.printf("\r\n");}
}

int main()
{
    M0=0;
    for(int i=1;i<9;i++){
        D_now[i]=D_tar[i]-5;
    }
    sv_set_neutral();
    
    sv1.period_ms(sv_cycle);
    sv2.period_ms(sv_cycle);
    sv3.period_ms(sv_cycle);
    sv4.period_ms(sv_cycle);
    sv5.period_ms(sv_cycle);
    sv6.period_ms(sv_cycle);
    sv7.period_ms(sv_cycle);
    sv8.period_ms(sv_cycle);
    
    /* 角度制御 １度ずつ変わる */
    ///*
    sb1.attach(&sb1_func,1.0f/kata_speed);
    sb2.attach(&sb2_func,1.0f/hiji_speed);
    sb3.attach(&sb3_func,1.0f/teku_speed);
    sb4.attach(&sb4_func,1.0f/kata_speed);
    sb5.attach(&sb5_func,1.0f/hiji_speed);
    sb6.attach(&sb6_func,1.0f/teku_speed);
    sb7.attach(&sb7_func,1.0f/kubi_speed);
    sb8.attach(&sb8_func,1.0f/kubi_speed);
    //*/
    

    /* 角度制御 1usずつ変わる */
    /*
    sb0.attach(&sb0_funcP,1.0f/s0_speed);
    sb1.attach(&sb1_funcP,1.0f/s1_speed);
    sb2.attach(&sb2_funcP,1.0f/s2_speed);
    sb3.attach(&sb3_funcP,1.0f/s3_speed);
    sb4.attach(&sb4_funcP,1.0f/s4_speed);
    sb5.attach(&sb5_funcP,1.0f/s5_speed);
    sb6.attach(&sb6_funcP,1.0f/s6_speed);
    sb7.attach(&sb7_funcP,1.0f/s7_speed);
    */
    
    sv_set_neutral();
    
    
    pc.baud(921600);
    //pc.baud(9600);
    pc.format(9,Serial::Odd,1);
    s1.format(9,Serial::Odd,1);
    xb.baud(9600);
    xb.format(9,Serial::Odd,1);
    /*
        pc.format(8,Serial::None,1);
    s1.format(8,Serial::None,1);
    xb.format(8,Serial::None,1);
    */
        pc.attach(&pc_read, Serial::RxIrq);
    s1.attach(&s1_read, Serial::RxIrq);
    xb.attach(&xb_read, Serial::RxIrq);
    
    md_wd[0]='Q';
    D_set(7,90);

    /* 権藤のスペース */
    /*
    gon.frequency(100*1000);
    gon_trans(gon_all ,gon_all ,gon_all);
    wait(1.5);
    gon_trans(gon_none,gon_none,gon_all);
    wait(1.5);
    gon_trans(gon_none,gon_all ,gon_all);
    wait(0.5);
    gon_trans(gon_all ,gon_none,gon_none);
    */
      
    /*
    while(1){
        M_for();
        wait(5);
        M_stp();
        wait(1);
        M_rev();
        wait(5);
        M_stp();
        wait(1);
}
*/
/*
    gon_trans(gon_ue,gon_ue2,gon_none);
    wait(1.5);
    gon_trans(gon_none,gon_sita,gon_sita2);
    wait(1.5);
    gon_trans(gon_migi,gon_migi,gon_migi);
    wait(1.5);
    gon_trans(gon_none,gon_hidari,gon_hidari);
    wait(1.5);
    gon_trans(gon_none,gon_none,gon_magao);
    wait(1.5);
    */
    //gon_trans(gon_all,gon_all,gon_all);
    
    int count = 1;
    gon_wink();
    while(1){
            if( mode != oldmode ){
                mode_change(mode);
                oldmode = mode;
                switch(mode)
                {
                    case 0:
                        pc.printf("can not judge this string.\r\n");
                        break;
                    
                    case 'a':   //名刺を渡す。
                        sv_set_pass();
                        break;
                    
                    case 'b':   //名刺を渡さない。ホームポジション
                        sv_set_neutral();
                        break;
                                        
                    case 'c':   //四秒間前進する。
                        M_for();
                        //Mt.attach(&M_stp,4.0f);
                        break;
                    
                    case 'd':   //四秒間後退する。
                        M_rev();
                        //Mt.attach(&M_stp,4.0f);
                        break;
                        
                    case 'r':   //その場でR回転する。
                        M_R_turn();
                        //Mt.attach(&M_stp,4.0f);
                        break;
                    
                    case 'l':   //その場でL回転する。
                        M_L_turn();
                        //Mt.attach(&M_stp,4.0f);
                        break;
                    
                    case 'e':   //強制停止
                        M_brk();
                        //M_stp();
                        //Mt.attach(&M_brk,4.0f);
                        break;
                    
                    case 'f':
                        gon_wink();
                        break;
                        
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                    case ':':
                    case ';':
                    case '<':
                    case '=':
                    case '>':
                        kubi_set(mode);
                        break;
                    
                    case '?':
                        kubi_shuffle();
                        break;
                    case 'F':
                    /*
                        SS=!SS;
                        if(SS){pc.baud(921600);}
                        else {pc.baud(9600);}
                    */    break;
                    
                    case 'S':   //マニュアル調整
                        sv_set_manual();
                        break;
                    
                    case 'P':   //入力文字の表示
                        PR=!PR;
                        break;
                    
                    case 'Q':   //quit
                        break;
                    
                    case 'X':   //debug
                        sv_set_debug();
                        break;
                        
                    case 'N':   //debug
                        sv_set_chk();
                        break;
                    
                    
                    default:
                        pc.printf("This string is not set action.\r\n");
                        pc.printf("You forgot to set.\r\n");
                    
                        break;
                }
                mode = 'Q';
            }//if
    } //while
}


void R_for(){
    R1 = 1;
    R2 = 0;
}
void R_rev(){
    R1 = 0;
    R2 = 1;
}
void R_stp(){
    R1 = 0;
    R2 = 0;
}
void R_brk(){
    R1 = 1;
    R2 = 1;
}
void L_for(){
    L1 = 0;
    L2 = 1;
}
void L_rev(){
    L1 = 1;
    L2 = 0;
}
void L_stp(){
    L1 = 0;
    L2 = 0;
}
void L_brk(){
    L1 = 1;
    L2 = 1;
}
void M_for(){
    R_for();L_for();}
void M_rev(){
    R_rev();L_rev();}
void M_stp(){
    R_stp();L_stp();}
void M_brk(){
    R_brk();L_brk();}
void M_R_turn(){
    R_rev();L_for();}
void M_L_turn(){
    R_for();L_rev();}
    
void sv_set(int no, double deg){
    int p = map(deg ,0 ,180 ,P_min[no],P_max[no]);
        //pc.printf("%d\t%d\r\n",no,p);
    switch(no)
    {
        case 1:
        sv1.pulsewidth_us(p);
        break;
        
        case 2:
        sv2.pulsewidth_us(p);
        break;
        
        case 3:
        sv3.pulsewidth_us(p);
        break;
        
        case 4:
        sv4.pulsewidth_us(p);
        break;
        
        case 5:
        sv5.pulsewidth_us(p);
        break;
        
        case 6:
        sv6.pulsewidth_us(p);
        break;
        
        case 7:
        sv7.pulsewidth_us(p);
        break;
        
        case 8:
        sv8.pulsewidth_us(p);
        break;
    }
}
void sv_setP(int no, double pw){
    double p=constrain(pw,P_min[no],P_max[no]);
    //pc.printf("%d\t%d\r\n",no,p);
    switch(no)
    {
        case 1:
        sv1.pulsewidth_us(p);
        break;
        
        case 2:
        sv2.pulsewidth_us(p);
        break;
        
        case 3:
        sv3.pulsewidth_us(p);
        break;
        
        case 4:
        sv4.pulsewidth_us(p);
        break;
        
        case 5:
        sv5.pulsewidth_us(p);
        break;
        
        case 6:
        sv6.pulsewidth_us(p);
        break;
        
        case 7:
        sv7.pulsewidth_us(p);
        break;
        
        case 8:
        sv8.pulsewidth_us(p);
        break;
    }
}
void sv_set_neutral(){
    
    D_set(1,90);
    D_set(2,97);
    D_set(3,00);
    D_set(4,90);
    D_set(5,80);
    D_set(6,160);
    
}
void sv_set_pass(){
    D_set(1,150);
    D_set(2,30);
    D_set(3,100);
    D_set(4,45);
    D_set(5,130);
    D_set(6, 75);
    Mt.attach(&M_push,4.2);
}

void sv_set_debug(){
    pc.attach(NULL, Serial::RxIrq);
    s1.attach(NULL, Serial::RxIrq);
    xb.attach(NULL, Serial::RxIrq);
    int no,tmp;
    pc.printf("super debug mode\r\n");
    
    svno:   //サーボ番号の入力
    pc.printf("input servo No\r\n");
    pc.scanf("%d",&no);
    if(no > 8 | no < 0)
    {pc.printf("error\r\n");goto svno;}
    
    minpw:  //最少パルス幅の入力
    pc.printf("\r\ninput min pulse width\r\n");
    pc.printf("now min pulse width : %4d\r\n",P_min[no]);
    pc.scanf("%d",&tmp);
    if(tmp > 3000 | tmp < 1)
    {pc.printf("error\r\n");goto minpw;}
    P_min[no]=(double)tmp;
    
    maxpw:  //最大パルス幅の入力
    pc.printf("\r\ninput max pulse width\r\n");
    pc.printf("now max pulse width : %4d\r\n",P_max[no]);
    pc.scanf("%d",&tmp);
    if(tmp > 3000 | tmp < 1)
    {pc.printf("error\r\n");goto maxpw;}
    P_max[no]=(double)tmp;
    
    setang: //セットする角度の入力
    pc.printf("\r\ninput set angle\r\n");
    pc.scanf("%d",&tmp);
    if(tmp > 180 | tmp < 0)
    {pc.printf("error\r\n");goto setang;}
    D_set(no,(double)tmp);
    
    pc.printf("\r\nset degree:%3lf\t pw:%4lf",D_tar[no],P_tar[no]);
    wait(0.5f);
    pc.printf("\r\ndebug fin...\r\n");
    pc.attach(&pc_read, Serial::RxIrq);
    s1.attach(&s1_read, Serial::RxIrq);
    xb.attach(&xb_read, Serial::RxIrq);
}
void sv_set_manual(){
    pc.attach(NULL, Serial::RxIrq);
    s1.attach(NULL, Serial::RxIrq);
    xb.attach(NULL, Serial::RxIrq);
    char mode='0';
    int no=0;
    double tmp=9999;
    printf("manual set mode\r\n");
    
    moudo:  //モード 
        pc.printf("input 'P' or 'D' or 'M'\r\n");
        pc.scanf("%c",&mode);
        if( (mode == 'M') || (mode == 'm') )    goto motor;
        if( (mode != 'P')&&(mode != 'p')&&(mode != 'D')&&(mode != 'd'))
            goto moudo;
            
    svno:   //servo no.
        pc.printf("input servo no\r\n");
        pc.scanf("%d",&no);
        if(no<1 || no>8)
            goto svno;
        if( (mode == 'P') || (mode == 'p') )
            goto pwset;
        else 
            goto degset;
        
    pwset:  //pluse width
        pc.printf("input servo pulse width\r\n");
        pc.scanf("%lf",&tmp);
        if(tmp<P_min[no] || tmp>P_max[no])
            goto pwset;     
        P_set(no,tmp);
        goto fin;
    
    degset: //degree
        pc.printf("input servo degree\r\n");
        pc.scanf("%lf",&tmp);
        if(tmp<0 || tmp>180)
            goto degset;
        D_set(no,tmp);
        goto fin;
        
    motor:
                pc.printf("wait 4.0s");
        M_push();
        M_for();
        wait(1.0);
        M_rev();
        wait(1.0);
        M_R_turn();
        wait(1.0);
        M_L_turn();
        wait(1.0);
        M_stp();
        wait(1.0);
        M_brk();
                pc.printf("fin\r\n");
        goto fin;
        
    fin:
                if(no!=0){
                    double tar=map(P_tar[no],P_max[no],P_min[no],180,0);
                    printf("set servo %1d %5.2lf degree (%4lfus)",no,tar,P_tar[no]);  
                }                   
                serial_buf_reset();
                pc.attach(&pc_read, Serial::RxIrq);
                s1.attach(&s1_read, Serial::RxIrq);
                xb.attach(&xb_read, Serial::RxIrq);
}
void sv_set_chk(){
    pc.printf("\n\r");
    for(int i=1;i<9;i++){
        pc.printf("servo %d -> %f\r\n",i,D_tar[i]);
        }
    }

void M_push(){
    M0=1;
    Mt.attach(&M_pull,5.5);
    mtban=0;
}
void M_pull(){
    M0=0;
    mtban=1;
}
void sb1_func(){
    if(D_now[1] < D_tar[1] )
        sv_set(1,++D_now[1]);
    else if(D_now[1] > D_tar[1] )
        sv_set(1,--D_now[1]);
}
void sb2_func(){
    if(D_now[2] < D_tar[2] )
        sv_set(2,++D_now[2]);
    else if(D_now[2] > D_tar[2] )
        sv_set(2,--D_now[2]);
}
void sb3_func(){
    if(D_now[3] < D_tar[3] )
        sv_set(3,++D_now[3]);
    else if(D_now[3] > D_tar[3] )
        sv_set(3,--D_now[3]);
}
void sb4_func(){
    if(D_now[4] < D_tar[4] )
        sv_set(4,++D_now[4]);
    else if(D_now[4] > D_tar[4] )
        sv_set(4,--D_now[4]);
}
void sb5_func(){
    if(D_now[5] < D_tar[5] )
        sv_set(5,++D_now[5]);
    else if(D_now[5] > D_tar[5] )
        sv_set(5,--D_now[5]);
}
void sb6_func(){
    if(D_now[6] < D_tar[6] )
        sv_set(6,++D_now[6]);
    else if(D_now[6] > D_tar[6] )
        sv_set(6,--D_now[6]);
}
void sb7_func(){
    if(D_now[7] < D_tar[7] )
        sv_set(7,++D_now[7]);
    else if(D_now[7] > D_tar[7] )
        sv_set(7,--D_now[7]);
}
void sb8_func(){
    if(D_now[8] < D_tar[8] )
        sv_set(8,++D_now[8]);
    else if(D_now[8] > D_tar[8] )
        sv_set(8,--D_now[8]);
}
/*
void sb1_funcP(){
    if(P_now[1] < P_tar[1] )
        sv_setP(1,++P_now[1]);
    else if(P_now[1] > P_tar[1] )
        sv_setP(1,--P_now[1]);
}
void sb2_funcP(){
    if(P_now[2] < P_tar[2] )
        sv_setP(2,++P_now[2]);
    else if(P_now[2] > P_tar[2] )
        sv_setP(2,--P_now[2]);
}
void sb3_funcP(){
    if(P_now[3] < P_tar[3] )
        sv_setP(3,++P_now[3]);
    else if(P_now[3] > P_tar[3] )
        sv_setP(3,--P_now[3]);
}
void sb4_funcP(){
    if(P_now[4] < P_tar[4] )
        sv_setP(4,++P_now[4]);
    else if(P_now[4] > P_tar[4] )
        sv_setP(4,--P_now[4]);
}
void sb5_funcP(){
    if(P_now[5] < P_tar[5] )
        sv_setP(5,++P_now[5]);
    else if(P_now[5] > P_tar[5] )
        sv_setP(5,--P_now[5]);
}
void sb6_funcP(){
    if(P_now[6] < P_tar[6] )
        sv_setP(6,++P_now[6]);
    else if(P_now[6] > P_tar[6] )
        sv_setP(6,--P_now[6]);
}
void sb7_funcP(){
    if(P_now[7] < P_tar[7] )
        sv_setP(7,++P_now[7]);
    else if(P_now[7] > P_tar[7] )
        sv_setP(7,--P_now[7]);
}
void sb8_funcP(){
    if(P_now[8] < P_tar[8] )
        sv_setP(8,++P_now[8]);
    else if(P_now[8] > P_tar[8] )
        sv_setP(8,--P_now[8]);
}
*/


void pc_read(){
    pc_wd[pcc]=pc.getc();
    if(mtban){
    Mt.detach();
    Mt.attach(&serial_buf_reset,0.05f);
    pcc++;
    }
}
/*
void pc_read(){
        Mt.detach();
        Mt.attach(&serial_buf_reset,0.05f);
}
*/

void s1_read(){
    s1_wd[s1c]=s1.getc();
    Mt.detach();
    Mt.attach(&serial_buf_reset,0.3f);
    s1c++;
}
void xb_read(){
    xb_wd[xbc]=xb.getc();
    Mt.detach();
    Mt.attach(&serial_buf_reset,0.3f);
    xbc++;
}
void serial_buf_reset(){
    for(int i=0;i<buf_size+1;i++){
        md_wd[i]=0;
    }
    if(pcc != 0){
        pc_wd[pcc]=0;
        strcpy(md_wd,pc_wd);
    }
    else if (s1c != 0){
        s1_wd[s1c]=0;
        strcpy(md_wd,s1_wd);
    }
    else {
        xb_wd[xbc]=0;
        strcpy(md_wd,xb_wd);
    }
    pcc=0;s1c=0;xbc=0;printmd_wd();
    mode = mode_judge();
    for(int i=0;i<buf_size;i++){
        pc_wd[i]=0;
        s1_wd[i]=0;
        xb_wd[i]=0;
        md_wd[i]=0;
    }
}
void mode_change(char mode){
    if(mode != 'Q' && mode != 0){
    pc.printf("mode = %c\r\n",mode);
    s1.printf("mode = %c\r\n",mode);
    xb.printf("mode = %c\r\n",mode);
    }
}
void kubi_set(int temp){
    temp -= 48;
    int tate = temp/5;
    int yoko = temp%5;
    if(tate == 0){D_set(7,120);}
    else if(tate == 1){D_set(7, 105);}
    else if(tate == 2){D_set(7, 90);}
    
    if(yoko == 0){D_set(8, 135);}
    else if(yoko == 1){D_set(8, 115);}
    else if(yoko == 2){D_set(8,  90);}
    else if(yoko == 3){D_set(8,  75);}
    else if(yoko == 4){D_set(8,  55);}
    if(temp==15){kubi_shuffle();}
    }
void kubi_shuffle(){
    for(int i=0 ;i<15;i++){
        int temp = rand()%16;
        kubi_set(i+48);
        wait(1.5);
        }
    }
void gon_trans(char red[8],char green[8],char blue[8]){

    for(int i=0;i<8;i++){
        gon_data[i]=red[i];
        gon_data[i+8]=green[i];
        gon_data[i+16]=blue[i];
        }
    gon.write(gon_address,gon_data,24);
    }
void gon_clear(){
    gon_trans(gon_none,gon_none,gon_none);
}
void gon_wink(){
    gon_trans(gon_none,gon_me2,gon_me1);
    wait(0.4);
    gon_trans(gon_none,gon_mabataki02,gon_mabataki01);
    wait(0.05);
    gon_trans(gon_none,gon_mabataki12,gon_mabataki11);
    wait(0.05);
    gon_trans(gon_none,gon_mabataki22,gon_mabataki21);
    wait(0.1);
    gon_trans(gon_none,gon_mabataki12,gon_mabataki11);
    wait(0.05);
    gon_trans(gon_none,gon_mabataki02,gon_mabataki01);
    wait(0.05);
    gon_trans(gon_none,gon_me2,gon_me1);
}

char mode_judge(){
    if(strstr(md_wd,"nice") != NULL)//初めまして     
        return 'a';
    else if(strstr(md_wd,"A") != NULL)
        return 'a';
    
    else if(strstr(md_wd,"thank") != NULL)//ありがとう   
        return 'b';
    else if(strstr(md_wd,"B") != NULL)
        return 'b';
    
    else if(strstr(md_wd,"forward") != NULL)//前進    
        return 'c';
    else if(strstr(md_wd,"C") != NULL)
        return 'c';
    
    else if(strstr(md_wd,"return") != NULL)//後進    
        return 'd';
    else if(strstr(md_wd,"D") != NULL)
        return 'd';   
         
    else if(strstr(md_wd,"R") != NULL)//右にその場で回転
        return 'r';
    
    else if(strstr(md_wd,"L") != NULL)//左にその場で回転
        return 'l';
    
    else if(strstr(md_wd,"freeze") != NULL)//停止→ロック   
        return 'e'; 
    else if(strstr(md_wd,"E") != NULL)
        return 'e';
        
    else if(strstr(md_wd,"F") != NULL)
        return 'f';
            
    else if(strstr(md_wd,"10") != NULL)
        return ':';    
    else if(strstr(md_wd,":") != NULL)   
        return ':'; 
    else if(strstr(md_wd,"11") != NULL)
        return ';';
    else if(strstr(md_wd,";") != NULL)  
        return ';'; 
    else if(strstr(md_wd,"12") != NULL)  
        return '<'; 
    else if(strstr(md_wd,"<") != NULL)
        return '<';    
    else if(strstr(md_wd,"13") != NULL)   
        return '='; 
    else if(strstr(md_wd,"=") != NULL)
        return '=';
    else if(strstr(md_wd,"14") != NULL)  
        return '>'; 
    else if(strstr(md_wd,">") != NULL)  
        return '>'; 
    else if(strstr(md_wd,"15") != NULL)   
        return '?'; 
    else if(strstr(md_wd,"?") != NULL)   
        return '?'; 
    else if(strstr(md_wd,"0") != NULL)   
        return '0'; 
    else if(strstr(md_wd,"1") != NULL)
        return '1';    
    else if(strstr(md_wd,"2") != NULL)   
        return '2'; 
    else if(strstr(md_wd,"3") != NULL)
        return '3';
    else if(strstr(md_wd,"4") != NULL)   
        return '4'; 
    else if(strstr(md_wd,"5") != NULL)
        return '5';    
    else if(strstr(md_wd,"6") != NULL)   
        return '6'; 
    else if(strstr(md_wd,"7") != NULL)
        return '7';
    else if(strstr(md_wd,"8") != NULL)  
        return '8'; 
    else if(strstr(md_wd,"9") != NULL)  
        return '9'; 
            
    else if(strstr(md_wd,"SS") != NULL)//スピードアップ        
        return 'F';
    else if(strstr(md_wd,"S") != NULL)//マニュアル調整モード        
        return 'S';
    
    else if(strstr(md_wd,"debug") != NULL)//デバッグモード
        return 'X';
    else if(strstr(md_wd,"X") != NULL)
        return 'X';
    
    else if(strstr(md_wd,"P") != NULL)//入力文字の表示
        return 'P';
    else if(strstr(md_wd,"print") != NULL)
        return 'P';
        
    else if(strstr(md_wd,"N") != NULL)
        return 'N'; 
    
    else if(strstr(md_wd,"Q") != NULL)
        return 'Q'; 
        
    return 0;
}