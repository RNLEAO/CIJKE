#include "headfile.h"
#include "control.h"



int sample_count = 0;
int state = 0;


float gyro_data[1] = {0.0f};

float roll_accel=0,pitch_accel=0;
float gyro_roll=0,gyro_pitch=0;

float roll = 0.0f, pitch= 0.0f;
float Angle_x=0;//闂佽瀛╅鏍疮椤愶箑绀冩い顓熷灥閺佷粙姊绘担鍛婂暈閻㈩垱顨婂畷鏇㈡偂楠炵喎缍婂畷鍗炩枎閹邦剚鐎梻浣告贡閸庛倝宕归悽鍓叉晩濠电姴娲﹂悡?
/************闂備浇宕甸崰鎰版偡閿旂偓鏆滈柟鐑樻煛閸嬫挾鎲撮崟顐熸灆闂佽鍠楁繛濠囥€佸☉姗嗘僵妞ゆ帊绶″Λ鐔兼⒒?*************/
float current_angle=0;//闂佽崵鍠愮划搴㈡櫠濡ゅ懎绠伴柛娑橈攻濞呯娀鏌ｅΟ铏逛粵闁哥姴妫濋弻鐔兼倻濡櫣浠村?
float target_angle=0;//闂傚倷鑳堕崕鐢稿疾閳哄懎绐楁俊銈呮噺閸嬪鏌ㄥ┑鍡樺闁哥姴妫濋弻鐔兼倻濡櫣浠村?
float angle_error;//闂備浇宕垫慨鏉懨归崒鐐插瀭闁哄顕?
float turn_control_output;
float base_speed = 0.0f; // 闂傚倷绀侀幉锟犫€﹂崶顒€绐楅幖绮规閼板潡鎮楀☉娅虫垶鎱ㄥ鍕╀簻闁瑰搫妫楁禍楣冩⒑鐠団€虫灍缂侇喗鎹囧顐㈩吋閸ワ附鍕冮梺浼欑到閼活垶寮妶鍥╃＜缂備降鍨归獮鎰版煕鐎ｎ偅宕屾慨濠冩そ椤㈡洟濡堕崨顔锯偓濠氭煟鎼达絾鍤€缁剧虎鍙冮獮蹇曗偓锝庡枛閻鏌曟径鍫濆妞?0



/************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨妯挎硾缁€澶愭煕濞戞瑦缍戦柣顓燁殜閺屾稓浠﹂崜褉濮囧┑?*************/
float L_raw = 0,R_raw = 0;//闂傚倷绀侀幉锟犳嚌妤ｅ啯鐓€闁挎繂鎷嬮悞钘壝归崗鍏肩稇闁绘劕锕ョ换娑㈠箣閻戝棙鐤侀梺鎼炲€曢澶婎嚕閸洖鐓涢柛瀣仛閸ㄥ潡骞嗛埀顒勬煕濞戞鎽犻柛瀣ㄥ姂閺屾洘绻涢崹顔煎闂?
float LM_raw = 0,RM_raw = 0;//闂傚倷绀侀幉锟犳嚌妤ｅ啯鐓€闁挎繂鎷嬮悞钘壝归崗鍏肩稇闁绘劕锕ョ换娑㈠箣閻戝棙鐤侀梺鎼炲€曢澶婎嚕閸洖鐓涢柛瀣仛閸ㄥ潡骞嗛埀顒勬煕濞戞鎽犻柛瀣ㄥ姂閺屾洘绻涢崹顔煎闂?

float L = 0,R = 0,LM = 0,RM = 0,MID = 0;//闂傚倷绀侀幉锟犳嚌妤ｅ啯鐓€闁挎繂鎷嬮悞钘壝归崗鍏肩稇闁绘劕锕ョ换娑㈠箣閻戝棙鐤侀梺鎼炲€曢澶婎嚕閸洖鐓涢柛瀣仛閸ㄥ潡骞嗛埀顒勬煕濞戞鎽犻柛瀣ㄥ姂閺屾洘绻涢崹顔煎闂?
uint16  max_AD = 998,min_AD = 1;//10闂傚倷绀侀幉锛勬暜閹烘嚦娑橆煥閸曗晙绮撻梺褰掓？閻掞箓宕戝┑瀣厱闁靛鍨哄▍鍡涙煛閸″繑娅婇柡宀€鍠栭獮宥夘敊閸忚偐鈧儳鈹戦悙瀛樺磩闁糕晜鐗曢銉╁礋椤曞懏些闂備線鈧偛鑻晶鎻掝熆閻熸壆澧︽い銏＄懇瀵挳鎮㈢悰鈩冪亙?98 12婵犵數鍋犻幓顏嗗緤閻ｅ瞼鐭撶€规洖娲︾€?095 

uint16  i = 0,j = 0,k1 = 0,temp = 0;

/**************************************************************************
闂傚倷绀侀幉锟犲垂閸忓吋鍙忛柕鍫濐槸濮规煡鏌ｉ弮鍌氬付缂佲偓婢舵劖鐓熼柡鍐ㄦ祩閸ゆ瑩鏌涘Ο缁樺唉闁?-- 闂備礁鎼ˇ顖炴偋閸℃稑绠犻幖娣灪閸欏繑銇勮箛鎾跺缂佲偓閸曨垱鐓犻柟顓熷笒閸旀粍绻涢崼婵堝煟闁哄被鍔岄埥澶娾枎鎼存繂顒㈡俊鐐€栧鐟懊哄鍛潟闁哄啫鐗滈弫濠囨⒒閸屾凹鍤熸い鏂挎濮婃椽宕ㄦ繝鍕櫑闂佹眹鍊曞ú顓㈠箖閸ф骞㈡繛鎴炵懃閸擃喖顪冮妶鍡樷拻闁哄拋鍋婂畷瑙勭附閸涘ň鎷哄銈嗗姂閸婃洟寮告惔銊﹀€?---
闂傚倷鑳堕…鍫㈡崲閸儱纾块弶鍫亖娴滆绻涢幋娆忕仼闁活厽顨婇弻娑氫沪閸撗€濮囧┑鐐茬墕閻栧ジ寮婚妸銉㈡婵炲棙鍩堥弳鈥斥攽?
闂備礁鎼ˇ顐﹀疾濠婂牆钃熼柕濞垮剭? 闂傚倷鑳堕…鍫ユ晝閿曞倸违閻庯綆鍓氶～鏇㈡煟閵婏缚娣?
**************************************************************************/
void set_target_speeds(float left_target, float right_target) {
    // 闂備礁鎼ˇ顐﹀疾濠婂牊鍋￠柍鍝勬噹闂傤垶鏌ｉ幋锝嗩棄闁活厽顨嗛妵鍕冀閵娧勫櫏缂備降鍔嬮崡鎶藉蓟濞戙垹绫嶉柛灞绢嚧閵忋倖鐓曟繛鍡楃箲椤ョ娀鏌嶈閸撶喎顭囪閿曘垽宕￠悘鑽ゅ劋鐎靛ジ寮堕幋婵嗘暏闂備焦鎮堕崕娲礂濮椻偓瀹曟垿骞樼紒妯轰簻閻庣懓瀚伴崑濠囧船濞差亝鈷戞慨鐟版搐婵″ジ鎮楀闂寸敖缂侇噮鍙冨畷鎺戔槈濮橈絾鐏冮梺璇插嚱缂嶅棝宕板Δ鍜佹晛婵炲棙鎸婚埛鎺楁煕閺囥劌澧┑顔兼喘閺屾盯鍩￠崒銈嗙杹闂佽桨绀佺粔鑸电閿曞倹鍤冮柍琛″亾缂佹唻绲剧换婵堝枈濡嘲浜鹃柛鎰ゴ閸嬫挻顦版惔锝囩暥婵犮垼鍩栭崝鏇犵矆閸屾褰掓晲閸モ晜鎲橀梺鍝ュ枎閹冲酣婀侀梺鎸庣箓閹虫劙宕欓崷顓犵＜?
    L_pid.Target = left_target;
    R_pid.Target = right_target;
}

/**************************************************************************
闂傚倷绀侀幉锟犲垂閸忓吋鍙忛柕鍫濐槸濮规煡鏌ｉ弮鍌氬付缂佲偓婢舵劖鐓熼柡鍐ㄦ祩閸ゆ瑩鏌涘Ο缁樺唉闁哄被鍔岄埥澶娾枎閹烘埈妫熸俊鐐€х粻鎺戠暦閻㈢绠悗锝庡枛缁犳氨鎲稿澶婄妞わ附娼抎闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柛宀€鍎戠紞鏍煙閻楀牊绶茬紒鈧?闂傚倷鑳堕…鍫㈡崲閸儱纾块弶鍫亖娴滆绻涢幋娆忕仼闁活厽顨婇弻娑氫沪閸撗€濮囧┑鐐茬墕閻栧ジ寮婚妸銉㈡婵炲棙鍩堥弳鈥斥攽?
闂備礁鎼ˇ顐﹀疾濠婂牆钃熼柕濞垮剭? 闂傚倷鑳堕…鍫ユ晝閿曞倸违閻庯綆鍓氶～鏇㈡煟閵婏缚娣?
**************************************************************************/
void init(void)
{

		adc_init(ADC_P00, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P01, ADC_SYSclk_DIV_2);	
		adc_init(ADC_P10, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P05, ADC_SYSclk_DIV_2);	  
		adc_init(ADC_P06, ADC_SYSclk_DIV_2);	  
		delay_ms(10);

		ctimer_count_init(MOTOR1_ENCODER);
		ctimer_count_init(MOTOR2_ENCODER);
		delay_ms(10);

		pwm_init(PWMA_CH2P_P62, 17000, 0);
		pwm_init(PWMA_CH1P_P60, 17000, 0);
		pwm_init(PWMA_CH4P_P66, 17000, 0);
		pwm_init(PWMA_CH3P_P64, 17000, 0);
		delay_ms(10);
		
		wireless_uart_init();
		
			
		gpio_mode(P2_6,GPO_PP);
		gpio_mode(P7_4,GPO_PP);
		gpio_mode(P0_7,GPO_PP);
		gpio_mode(P5_2,GPO_PP);

		// Keys: configure as quasi-bidirectional inputs with weak pull-up.
		gpio_mode(P4_5,GPIO);
		gpio_mode(P7_0,GPIO);
		gpio_mode(P7_1,GPIO);
		gpio_mode(P7_2,GPIO);
		gpio_mode(P7_3,GPIO);
		gpio_mode(P7_5,GPIO);
		P45 = 1;
		P70 = 1;
		P71 = 1;
		P72 = 1;
		P73 = 1;
		P75 = 1;


}


/**************************************************************************
闂傚倷绀佸﹢閬嶁€﹂崼銉嬪洭鎮界粙鎸庣€銈嗘尵閸婏絽鈽夐姀鐘靛姦濡炪倖宸婚崑鎾绘婢舵劖鍊甸柨婵嗛閸樻挳鏌＄仦璇测偓婵嬬嵁?闂備浇顕х€涒晠宕橀懡銈囩＝婵ê宕慨顒勬煕閺囥劌鐏犵紒?mode:
闂傚倷鑳堕、濠囨儗閸ヮ剙绀冮柕濞у啫绠戦梻鍌欑濠€閬嶁€﹂崼銉嬪洭鎮界粙鎸庣€銈嗘尪閸ㄦ椽宕戦幇鐗堢厾闁告縿鍎洪弳顖炴煕鐎ｎ偅灏伴柟宄版噽閹叉挳宕熼幋顖滅暤闁哄本绋戦～婵嬪礈瑜忛弳銈夋⒑闂堚晝鍒伴柟鑺ョ矌濡叉劙寮崼顐ｆ櫓闁荤偞绋堥埀顒€鍘栨竟?mode = 1: 闂傚倸鍊烽悞锕併亹閸愵亞鐭撻柛顐ｆ礃閸嬵亪鏌涢埄鍐槈缂佺姵婢橀…鍧楁嚋闂堟稑顫嶉梺鍝勬閻╊垶寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鍛婁繆閻愭潙绗ч柟宄扮秺閹垽鎮℃惔锛勫綁婵＄偑鍊曠换鎰板箠鎼淬劍鍎婇悹鍥ㄧゴ濡插牓鏌熼悙顒€澧柛搴＄Ч閺屽秷顧侀柛鎾寸懇瀹曟劙骞栨担鐟颁痪闂佸憡娲﹂崹閬嶅磻濠靛枹褰掓偂鎼达絾鎲煎┑顕嗙稻閸旀瑩寮?mode = 0: 濠电姵顔栭崰妤冩崲閹邦喖绶ゅù鐘差儐閸嬪鎮规潪鎵Э婵炴垯鍨归悞鍨亜閹哄棗浜鹃梻鍥ь樀閹鏁愭惔鈥茬盎闂佷紮绠戦…宄邦潖閸濆嫧鏋庨柟顖嗗嫮浜惧┑鐘愁問閸犳骞冮崒鐐靛祦闁?
**************************************************************************/



// --- Key Scan Function (Provided by User - Assumed Unchanged) ---
static volatile uint8 ui_key_event_pending = KEY_EVENT_NONE;

#define UI_DEBOUNCE_TICKS       3
#define UI_LONG_PRESS_TICKS   200

static void post_ui_key_event(uint8 event)
{
    if ((event != KEY_EVENT_NONE) && (ui_key_event_pending == KEY_EVENT_NONE))
    {
        ui_key_event_pending = event;
    }
}

uint8 fetch_ui_key_event(void)
{
    uint8 event = ui_key_event_pending;
    ui_key_event_pending = KEY_EVENT_NONE;
    return event;
}

uint8 key_scan(int mode)
{
    static unsigned int key_pressed_state = 1;
    if (mode) { key_pressed_state = 1; }

    if (key_pressed_state == 1 && (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0))
    {
        delay_ms(10);
        if (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0)
        {
            key_pressed_state = 0;

            if (P70 == 0) return KEY_EVENT_PAGE_PREV;
            if (P71 == 0) return KEY_EVENT_PAGE_NEXT;
            if (P73 == 0) return KEY_EVENT_ITEM_NEXT;
            if (P72 == 0)
            {
                if (P75 == 0) return KEY_EVENT_ADJ_DEC;
                return KEY_EVENT_ADJ_INC;
            }
            return KEY_EVENT_NONE;

        }
    }
    else if (key_pressed_state == 0 && (P70 == 1 && P71 == 1 && P72 == 1 && P73 == 1))
    {
        key_pressed_state = 1;
    }

    return KEY_EVENT_NONE;
}
/**************************************************************************
pwm缂傚倸鍊风粈渚€藝娴犲鍨傚ù鍏兼綑閸ㄥ倿骞栫划瑙勵潑闁搞倖娲熼弻宥堫檨闁告挾鍠庨悾宄拔旈崘顏堚攺闂侀潻瀵岄崢楣兯囬鈧娲捶椤撗呭姼濡炪倧濡囬弫璇差嚕椤愶富鏁囬柣鎰Ф閸犳牠宕洪崟顖氱闁靛ě鍐唹闂傚倷娴囬～澶愭倶濮樿泛绀夐幖娣妽閸嬪绻濇繝鍌氭灓闁搞倖娲熼弻宥堫檨闁告挾鍠庨悾?mode:
闂傚倷鑳堕、濠囨儗閸ヮ剙绀冮柕濞у啫绠戦梻鍌欑濠€閬嶁€﹂崼銉嬪洭鎮界粙鎸庣€銈嗘尪閸ㄦ椽宕戦幇鐗堢厾闁告縿鍎洪弳顖炴煕鐎ｎ偅灏伴柟宄版噽閹叉挳宕熼幋顖滅暤闁哄本绋戦～婵嬪礈瑜忛弳銈夋⒑闂堚晝鍒伴柟鑺ョ矌濡叉劙寮崼顐ｆ櫓闁荤偞绋堥埀顒€鍘栨竟?mode = 1: 闂傚倸鍊烽悞锕併亹閸愵亞鐭撻柛顐ｆ礃閸嬵亪鏌涢埄鍐槈缂佺姵婢橀…鍧楁嚋闂堟稑顫嶉梺鍝勬閻╊垶寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鍛婁繆閻愭潙绗ч柟宄扮秺閹垽鎮℃惔锛勫綁婵＄偑鍊曠换鎰板箠鎼淬劍鍎婇悹鍥ㄧゴ濡插牓鏌熼悙顒€澧柛搴＄Ч閺屽秷顧侀柛鎾寸懇瀹曟劙骞栨担鐟颁痪闂佸憡娲﹂崹閬嶅磻濠靛枹褰掓偂鎼达絾鎲煎┑顕嗙稻閸旀瑩寮?mode = 0: 濠电姵顔栭崰妤冩崲閹邦喖绶ゅù鐘差儐閸嬪鎮规潪鎵Э婵炴垯鍨归悞鍨亜閹哄棗浜鹃梻鍥ь樀閹鏁愭惔鈥茬盎闂佷紮绠戦…宄邦潖閸濆嫧鏋庨柟顖嗗嫮浜惧┑鐘愁問閸犳骞冮崒鐐靛祦闁?
**************************************************************************/
uint8 current_key=0;
uint8 last_key_state =0;
uint8 pwm_state = 0;
char pwm_state_charge = 255;

uint8 Pwmout =0;
void key_scan_cycle_pwm_state(void)
{
    static uint8 raw70 = 0, stable70 = 0, cnt70 = 0;
    static uint8 raw71 = 0, stable71 = 0, cnt71 = 0;
    static uint8 raw72 = 0, stable72 = 0, cnt72 = 0;
    static uint8 raw73 = 0, stable73 = 0, cnt73 = 0;
    static uint8 raw75 = 0, stable75 = 0, cnt75 = 0;

    static uint8 prev70_down = 0;
    static uint8 prev71_down = 0;
    static uint8 prev72_down = 0;
    static uint8 prev73_down = 0;

    static uint8 key70_long_sent = 0;
    static uint8 key73_long_sent = 0;
    static uint8 combo73_used = 0;
    static uint8 combo70_used = 0;
    static uint8 combo73_long_sent = 0;

    static uint16 key70_ticks = 0;
    static uint16 key73_ticks = 0;
    static uint16 combo73_ticks = 0;

    uint8 cur70 = (P70 == 0) ? 1 : 0;
    uint8 cur71 = (P71 == 0) ? 1 : 0;
    uint8 cur72 = (P72 == 0) ? 1 : 0;
    uint8 cur73 = (P73 == 0) ? 1 : 0;
    uint8 cur75 = (P75 == 0) ? 1 : 0;

    if (cur70 != raw70) { raw70 = cur70; cnt70 = UI_DEBOUNCE_TICKS; }
    else if (cnt70 > 0) { cnt70--; if (cnt70 == 0) stable70 = raw70; }

    if (cur71 != raw71) { raw71 = cur71; cnt71 = UI_DEBOUNCE_TICKS; }
    else if (cnt71 > 0) { cnt71--; if (cnt71 == 0) stable71 = raw71; }

    if (cur72 != raw72) { raw72 = cur72; cnt72 = UI_DEBOUNCE_TICKS; }
    else if (cnt72 > 0) { cnt72--; if (cnt72 == 0) stable72 = raw72; }

    if (cur73 != raw73) { raw73 = cur73; cnt73 = UI_DEBOUNCE_TICKS; }
    else if (cnt73 > 0) { cnt73--; if (cnt73 == 0) stable73 = raw73; }

    if (cur75 != raw75) { raw75 = cur75; cnt75 = UI_DEBOUNCE_TICKS; }
    else if (cnt75 > 0) { cnt75--; if (cnt75 == 0) stable75 = raw75; }

    if (stable72 && !prev72_down)
    {
        if (stable75) post_ui_key_event(KEY_EVENT_ADJ_DEC);
        else post_ui_key_event(KEY_EVENT_ADJ_INC);
    }

    if (stable71 && !prev71_down)
    {
        post_ui_key_event(KEY_EVENT_PAGE_NEXT);
    }

    if (stable70 && stable73)
    {
        if (!prev70_down || !prev73_down)
        {
            combo73_ticks = 0;
            combo70_used = 0;
            combo73_used = 0;
            combo73_long_sent = 0;
        }
        else if (combo73_ticks < UI_LONG_PRESS_TICKS)
        {
            combo73_ticks++;
        }

        if (combo73_ticks >= UI_LONG_PRESS_TICKS && !combo73_long_sent)
        {
            combo73_long_sent = 1;
            combo70_used = 1;
            combo73_used = 1;
            key70_long_sent = 1;
            key73_long_sent = 1;
            pwm_state = 2;
            Pwmout = pwm_state;
            post_ui_key_event(KEY_EVENT_ENTER_CLEAN);
        }
    }
    else
    {
        combo73_ticks = 0;
        combo73_long_sent = 0;
    }

    if (stable70 && !stable73)
    {
        if (!prev70_down)
        {
            key70_ticks = 0;
            key70_long_sent = 0;
            combo70_used = 0;
        }
        else if (key70_ticks < UI_LONG_PRESS_TICKS)
        {
            key70_ticks++;
        }

        if (key70_ticks >= UI_LONG_PRESS_TICKS && !key70_long_sent)
        {
            key70_long_sent = 1;
            if (pwm_state == 2) pwm_state = 0;
            else pwm_state = (pwm_state == 1) ? 0 : 1;
            Pwmout = pwm_state;
            post_ui_key_event(KEY_EVENT_RUN_TOGGLE);
        }
    }
    else if (prev70_down)
    {
        if (!key70_long_sent && !combo70_used && (key70_ticks > 0))
        {
            post_ui_key_event(KEY_EVENT_PAGE_PREV);
        }
        key70_ticks = 0;
        key70_long_sent = 0;
        combo70_used = 0;
    }

    if (stable73 && !stable70)
    {
        if (!prev73_down)
        {
            key73_ticks = 0;
            key73_long_sent = 0;
            combo73_used = 0;
        }
        else if (key73_ticks < UI_LONG_PRESS_TICKS)
        {
            key73_ticks++;
        }

        if (key73_ticks >= UI_LONG_PRESS_TICKS && !key73_long_sent)
        {
            key73_long_sent = 1;
            post_ui_key_event(KEY_EVENT_SAVE_ALL);
        }
    }
    else if (prev73_down)
    {
        if (!key73_long_sent && !combo73_used && (key73_ticks > 0))
        {
            post_ui_key_event(KEY_EVENT_ITEM_NEXT);
        }
        key73_ticks = 0;
        key73_long_sent = 0;
        combo73_used = 0;
    }

    prev70_down = stable70;
    prev71_down = stable71;
    prev72_down = stable72;
    prev73_down = stable73;
}

static float get_step_scale_from_switch(void)
{
    if (P76 == 0)
        return 50.0f;
    else
        return 1.0f;
}

void adjust_parameter_by_key_float(uint8 key_value, float *parameter, float step)
{
    float real_step = step * get_step_scale_from_switch();

    if (key_value == KEY_EVENT_ADJ_INC)
    {
        *parameter += real_step;
    }
    else if (key_value == KEY_EVENT_ADJ_DEC)
    {
        *parameter -= real_step;
    }
}

float low_pass_filter(float current_value, float last_value, float alpha)
{
    return current_value * alpha + last_value * (1 - alpha);
}

void change_speed_Target(int speed)
{
    L_pid.Target = speed;
    R_pid.Target = speed;
}

void change_speed_Target_base(int speed)
{
    L_pid.Target_base = speed;
    R_pid.Target_base = speed;
}





float limit_range(float input, float limit)
{
    if (input > limit)
        return limit;
    else if (input < -limit)
        return -limit;
    else
        return input;
}





/**
 * @brief 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺嗐倕銆掑锝呬壕閻庤娲╃徊浠嬶綖濠婂牆鐒垫い鎺戝闂傤垱绻涘顔荤凹闁稿骸绉归弻娑㈠即閵娿儱绠婚梺鍛婎殕缁捇骞冨Δ鍛仺闁割煈鍋勫▓宀勬⒑闁偛鑻晶顕€鏌涘顒夊剰閸楅亶鏌熼梻瀵割槮閻熸瑱绠撻弻銊╁即濮樺崬濡介悷婊勫Ω閸ャ劎鍘遍梺鍦劋閺屻劑銆傛總鍛婄厱闁靛绲芥俊鍧楁煙婵傚摜鐣虹€规洏鍔戦獮宥夘敊閸撗勬瘒闂備浇宕垫慨鎶芥倿閿曞倸纾块柟鎯板Г閸嬧晛螖閿濆懎鏆為柛搴＄Ч閺屾盯寮撮妸銉т画濡炪値鍋勯惌鍌炲蓟濞戞瑧绡€闁稿本纰嶉崐顖氣攽閻愯泛浜归柡浣割煼閻涱噣骞掗弴銊π╅梻浣虹《閺呮粓銆冩繝鍥х閹兼番鍔岀壕濂告煟閹邦収鍟忔繛鎻掓健濮婃椽妫冮埡鍐偡闂佸憡鏌ㄩ敃顏堝春閳? *
 * 濠电姵顔栭崰妤冪紦閸ф纾归柡鍥ュ灩閸氬綊姊洪鈧粔鎾几娓氣偓閺岋絽螖閳ь剟鎮у鍐炬綎濡わ絽鍟悡娑㈡倶閻愰鍤欓柛鏂诲劤缁辨帗锛愬┑鍡楃睄閻庤娲橀〃濠冧繆閻戣棄唯闁靛／鍛呮粓姊?check_value 闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佸嚬閸犳岸宕氶幒鎿冩Ч閹煎瓨锚娴滈箖鏌ｉ姀鐘典粵闁搞倐鍋撻梻渚€鈧偛鑻晶鐗堛亜閹寸偞绀嬮柛鈹惧亾? * 婵犵數濮烽。浠嬪焵椤掆偓閸熷潡鍩€椤掆偓缂嶅﹪骞冨Ο璇茬窞鐎光偓閳ь剛澹曢崗鑲╃瘈闂傚牊绋掗敍宥夋煕閵堝洤鈻堥柡?>= abs_threshold闂傚倷鐒︾€笛呯矙閹达附鍤愭い鏍仜閸ㄥ倹銇勯弽顐粶缂佺姰鍎甸弻鐔煎箚瑜忛敍宥囩棯閹岀吋闁哄睙鍐炬僵妞ゆ巻鍋撻柍褜鍓濆▔鏇犲垝鐎ｎ厾鐤€婵炴垶锕㈠Λ?(闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸?P52 = 1)闂? * 婵犵數濮烽。浠嬪焵椤掆偓閸熷潡鍩€椤掆偓缂嶅﹪骞冨Ο璇茬窞鐎光偓閳ь剛澹曢崗鑲╃瘈闂傚牊绋掗敍宥夋煕閵堝洤鈻堥柡?< abs_threshold闂傚倷鐒︾€笛呯矙閹达附鍤愭い鏍仜閸ㄥ倹銇勯弽顐粶缂佲偓閸岀偞鐓欓柟顖涙緲琚氶梺鎸庣⊕濡啴寮昏椤繈顢楅埀顒勫焵椤掑啯纭剁紒顔碱儓缁犳稑鈽夊鈧Λ?(闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸?P52 = 0)闂? *
 * 闂傚倷绀侀幉锟犲箰閸濄儳鐭撳ù锝呭暔娴滅懓銆掑锝呬壕閻庢鍠栭悥濂稿垂妤ｅ啫鐭楀鑸得禒娲⒒? * - 闂傚倸鍊搁崐绋棵洪悩璇茬；闁瑰墽绮崑锟犳煛閸ャ劍鐨戞い锔肩畵閹宕归顒冣偓璺ㄢ偓?<math.h> 婵犵數濮伴崹鐓庘枖濞戙垺鍋ら柕濞炬櫅閸戠娀鏌涢幇銊︽珖妞も晝鍏樺Λ鍛搭敆娴ｆ亽鍋為梺? * - 闂傚倷绀侀幉锟犳偡閿曞倹鏅濋柕蹇嬪€曢梻?P52 闂傚倸顭崑鍕洪妶澶婄疇婵せ鍋撳┑锛勵棎缁犳盯骞欓崘銊︽珚闂備浇鍋愰埛鍫ュ礈濞戙垺鍎夐柛娑卞枟閸欏繑绻濋崹顐ｅ暗闁诡喖銈搁弻宥夊Ψ閿旇崵鍔烽梺閫炲苯澧叉い顐㈩樀閹矂顢欓崜褌绗夐梺缁樻煥閹芥粓鍩㈤弮鍫熺厽闁瑰浼濋鍛浄婵せ鍋撻柡灞剧洴楠炴帡骞橀崘鎻捫︽繝鐢靛仦閹逛線锝炴径鎰ч柨鏇炲€归弲鏌ユ煕濞戝崬鏋ょ憸鐗堢矒濮婂搫效閸パ冾瀳闁诲孩鍑归崹鍐测枎閵忋値鏁囬柕蹇曞Х椤︹晠姊洪崨濠勭畵閻庢凹鍠氱划姘跺锤濡や胶鍘鹃柣搴㈢⊕椤洭宕懠顒傜＜閺夊牄鍔庣粻鐐烘煟濞戝崬娅嶇€殿喚鏁婚幃浠嬫偨閸偄娅?1 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮婚妷鈺傜厵闁诡垎鍜冪礊缂佺偓鍎抽…鐑藉蓟? 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮堥崒鐐寸厵闁诡垱婢樿闂佹寧绋掑Λ鍐蓟? *
 * @param check_value 闂備浇宕甸崰鎰洪幋锔藉殑閻犲搫銈藉ú顏勭妞ゆ棁鍋愰濠囨偡濠婂啰绠伴柍缁樻崌楠炲鏁冮埀顒勬儗濡や焦鍙忔俊鐐额嚙娴滈箖姊洪棃鈺冪ɑ婵＄偘绮欓獮鍡涘礃椤旇偐顦板銈嗗姉閸犲孩绂?(濠电姷鏁搁崑鐐哄箚瀹€鍕獥婵娉涚壕濠氭煥濠靛棭妲搁柛?闂? * @param abs_threshold 闂備浇宕甸崰鎰版偡鏉堚晝涓嶉柟杈剧祷娴滃綊鏌涘畝鈧崑鐐哄箟閵夆晜鐓曠€光偓閳ь剟宕戦悢绗衡偓鍛村箻椤旂晫鍘遍梺鎸庣箓閸燁偅淇婃總鍛婄厵濠靛倸澹婂Λ鎴︽煙缁夊棗瀚峰Σ褰掑箹濞ｎ剙鐏柡鍡欏Х缁辨捇宕掑▎鎴濆缂備浇顕ч崐鍧椼€佸棰濇晪闁逞屽墮閻ｇ兘骞囬鐘绘闂佸憡绋戦…顒€顭囬幘缁樷拺?(濠电姷鏁搁崑鐐哄箚瀹€鍕獥婵娉涚壕濠氭煥濠靛棭妲搁柛?闂? */
void buzzer_control_with_enable(float check_value, float abs_threshold, int enable_state)
{
    // 濠电姷顣藉Σ鍛村磻閳ь剟鏌涚€ｎ偅宕岄柡宀嬬磿娴狅妇鎷犻幓鎺戭潚闂佽崵鍋炴穱鐑樸仈閸濄儳鐭欏鑸靛姦閺佸倿鏌涘☉鍗炴灕闁瑰鍋呯换娑氣偓鐢殿焾闉嬪銈嗘⒐閸旀危?
    // 1. 闂傚倷绀侀幉锟犳偡閿曞倹鏅濋柕蹇嬪€曢梻顖涚箾瀹割喕绨奸柛搴＄Ч閺屾盯寮撮妸銉ョ闂佸憡顨嗙划鎾诲箖濡ゅ懏鍋ㄩ柛顭戝亜濞堝矂姊洪柅鐐茶嫰婢ь噣鏌涘顒夊剰閸楅亶鏌熼梻瀵割槮閻熸瑱绠撻弻銊╁即濮樺崬濡介悷婊勫Ω閸ャ劎鍘遍梺鐟版惈缁夋潙鐣甸崱妯圭箚闁圭粯甯楅崰妯尖偓?    // 2. 闂傚倷鑳堕崑鎾绘嚌妤ｅ啫纾归悹鎭掑妿閻鐓崶銊р槈闂傚偆鍨堕弻鏇熺節韫囨稒顎嶇紒妤佸灴濮婃椽宕ㄦ繝鍕殹缂備胶濮电敮锟犳晲閻愮儤鏅濋柛灞句亢琚濇俊鐐€栭悧妤冨垝瀹ュ姹?(enable_state == 1)
		#if 0
	    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1))
    {
        // 婵犵數鍋為崹鍫曞箰閸洖纾归柟鍓х帛閻撳倹绻濇繝鍌滃缂佺嫏鍥ㄧ厓闁靛鍎辩痪褎銇勯幇顏嗙煓婵﹥妞介、娆撳垂椤斞勬尵閳ь剚顔栭崯顐﹀川椤旇棄鍔氱紓鍌欑劍缁嬫垵锕㈡潏鈺冪焼濠㈣埖鍔栭悡娆撴煕椤垵浜濋悘蹇ｅ幘缁辨帗娼忛妸銉т紝闂佺妫勯柊锝呯暦閹烘鍊锋繛鍫熷缁绢垶姊?        P52 = 1; // 闂傚倷鑳堕…鍫ヮ敄閸涱劶娲煛閸滀焦鏅?P52 = 1 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮婚妷鈺傜厵闁诡垎鍜冪礊缂?
    }
    else
    {
        // 婵犵數鍋涢顓熸叏妤ｅ喚鏁婂┑鐘插缁€濠勨偓骞垮劚濡盯鍩㈤弮鍫熺厓鐟滄粓宕滃杈╃煓濠㈣埖鍔﹂弫鍌炴煕濞戝崬鏋﹂柟濂夊亝缁绘稓鈧數顭堟牎濠电姭鍋撻柛锔诲幘缁犳柨顭跨捄渚叕闁绘挶鍎靛浠嬪炊椤掆偓缁€?(缂傚倸鍊搁崐鐑芥倿閿旂偓宕查柛宀€鍎愰弫瀣亜閺囨浜鹃悗瑙勬礃閸旀瑩骞冮埄鍐╁劅闁挎繂妫涘Σ妤€鈹戦悙鑼憼缂侇喖鐭傞弻濠囨晲閸涘偊缍佸畷鍫曨敆閳ь剛澹?闂?enable_state 婵犵數鍋為崹鍫曞箰閸濄儳鐭撻梻鍫熷厷?1)闂傚倷鐒︾€笛呯矙閹达附鍤愭い鏍仜閻鎲搁弬娆炬綎濠电姵鍑归弫鍥煃閳轰礁鏆㈤柛姘€规穱濠囧Χ韫囨凹鍚呯紓浣藉蔼濡嫰鍩?
        P52 = 0; // 闂傚倷鑳堕…鍫ヮ敄閸涱劶娲煛閸滀焦鏅?P52 = 0 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮堥崒鐐寸厵闁诡垱婢樿闂?
    }
		#endif
		
		
		
		
		
		
		#if 1

		if ((check_value== abs_threshold) && (enable_state == 1))
    {
        // 婵犵數鍋為崹鍫曞箰閸洖纾归柟鍓х帛閻撳倹绻濇繝鍌滃缂佺嫏鍥ㄧ厓闁靛鍎辩痪褎銇勯幇顏嗙煓婵﹥妞介、娆撳垂椤斞勬尵閳ь剚顔栭崯顐﹀川椤旇棄鍔氱紓鍌欑劍缁嬫垵锕㈡潏鈺冪焼濠㈣埖鍔栭悡娆撴煕椤垵浜濋悘蹇ｅ幘缁辨帗娼忛妸銉т紝闂佺妫勯柊锝呯暦閹烘鍊锋繛鍫熷缁绢垶姊?        P52 = 1; // 闂傚倷鑳堕…鍫ヮ敄閸涱劶娲煛閸滀焦鏅?P52 = 1 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮婚妷鈺傜厵闁诡垎鍜冪礊缂?
    }
    else
    {
        // 婵犵數鍋涢顓熸叏妤ｅ喚鏁婂┑鐘插缁€濠勨偓骞垮劚濡盯鍩㈤弮鍫熺厓鐟滄粓宕滃杈╃煓濠㈣埖鍔﹂弫鍌炴煕濞戝崬鏋﹂柟濂夊亝缁绘稓鈧數顭堟牎濠电姭鍋撻柛锔诲幘缁犳柨顭跨捄渚叕闁绘挶鍎靛浠嬪炊椤掆偓缁€?(缂傚倸鍊搁崐鐑芥倿閿旂偓宕查柛宀€鍎愰弫瀣亜閺囨浜鹃悗瑙勬礃閸旀瑩骞冮埄鍐╁劅闁挎繂妫涘Σ妤€鈹戦悙鑼憼缂侇喖鐭傞弻濠囨晲閸涘偊缍佸畷鍫曨敆閳ь剛澹?闂?enable_state 婵犵數鍋為崹鍫曞箰閸濄儳鐭撻梻鍫熷厷?1)闂傚倷鐒︾€笛呯矙閹达附鍤愭い鏍仜閻鎲搁弬娆炬綎濠电姵鍑归弫鍥煃閳轰礁鏆㈤柛姘€规穱濠囧Χ韫囨凹鍚呯紓浣藉蔼濡嫰鍩?
        P52 = 0; // 闂傚倷鑳堕…鍫ヮ敄閸涱劶娲煛閸滀焦鏅?P52 = 0 闂備浇宕甸崑鐐电矙韫囨稑绀夐幖娣妼妗呭┑顔筋焾濞夋稓绮堥崒鐐寸厵闁诡垱婢樿闂?
    }
		#endif
		


}









	/*********************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨妯挎硾缁€澶愭煕濞戝崬鏋涚紒鍓佸仱閺屾盯寮撮妸銉ョ闂?********************//*********************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨妯挎硾缁€澶愭煕濞戝崬鏋涚紒鍓佸仱閺屾盯寮撮妸銉ョ闂?********************//*********************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨妯挎硾缁€澶愭煕濞戝崬鏋涚紒鍓佸仱閺屾盯寮撮妸銉ョ闂?********************/	






	/*********************闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡?********************//*********************闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡?********************//*********************闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡?********************/	


uint16 read_adc_average(ADCN_enum channel, unsigned short avg_times, ADCRES_enum resolution)
{
    unsigned long sum = 0;
    unsigned short adc_value = 0;
    unsigned short i;

    if (avg_times == 0) {
        avg_times = 1;
    }

    for (i = 0; i < avg_times; i++)
    {
        adc_value = adc_once(channel, resolution);

        sum += adc_value;
    }

    return (uint16)(sum / avg_times);
}

	/*********************闂傚倸鍊搁崐鍝モ偓姘煎弮瀹曟繈寮撮悩鍏哥瑝?********************//*********************闂傚倸鍊搁崐鍝モ偓姘煎弮瀹曟繈寮撮悩鍏哥瑝?********************//*********************闂傚倸鍊搁崐鍝モ偓姘煎弮瀹曟繈寮撮悩鍏哥瑝?********************/	


float limit_float(float value, float min_limit, float max_limit)
{
    float result;

    if (value < min_limit)
    {
        result = min_limit;
    }
    else if (value > max_limit)
    {
        result = max_limit;
    }
    else
    {
        result = value;
    }

    return result;
}

	/*********************闂佽崵鍠愮划搴㈡櫠濡ゅ啯鏆滃┑鐘插閸楁岸鏌熺紒銏犳灈缂佲偓?********************//*********************闂佽崵鍠愮划搴㈡櫠濡ゅ啯鏆滃┑鐘插閸楁岸鏌熺紒銏犳灈缂佲偓?********************//*********************闂佽崵鍠愮划搴㈡櫠濡ゅ啯鏆滃┑鐘插閸楁岸鏌熺紒銏犳灈缂佲偓?********************/	


float normalize_float(float value, float min, float max)
{
    float diff;
    float range;
    float result;

    if (max <= min)
    {
        return 0.0f;
    }
		//闂傚倷绀侀幉锛勬暜閹烘嚦娑樷枎閹炬潙鍓?
    range = max - min;
		//闂傚倷绀侀幉锛勬暜閹烘嚦娑樜旈崨顔间槐?
    diff = value - min;

    result = (diff * 100.0f) / range;

    // 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺?Dianci_Guiyi 闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佺煯閸楁娊宕洪埀顒併亜閹哄秶鍔嶆い顐ｎ殘缁辨帗锛愬┑鍡曠盎闁兼寧鍔欓幃褰掑炊閵娿儳绁峰銈庡亾缂嶄礁顫忔繝姘妞ゆ巻鍋撳┑顔兼川缁?
    if (result > 100.0f)
    {
        result = 100.0f;
    }

    if (result < 0.0f)
    {
        result = 0.0f;
    }

    return result;
}


/**
 * @brief 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺嗐倕霉閿濆懏璐￠柍缁樻⒐閵囧嫰骞橀崡鐐典痪闂佹寧绋撻崰鏍蓟閿濆顫呴柣妯哄悁缁敻姊洪崨濠冪叆闁硅櫕锚閻?(MID) 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晛鈹戦悩瀹犲缂佲偓婢舵劖鐓忓┑鐘茬箺閸氬倿鏌涚€ｎ偅灏伴柟宄版嚇閹粌螣閼测晛鐐婇梻鍌欑閹碱偆绮旈弶鎳ㄦ椽顢橀姀鐘靛姦濡炪倖甯掗崐鎼佸几鎼淬劍鍊甸悷娆忓閳绘洟鏌℃担鍝バゅù鐙呯畵閹崇偤濡烽妷锔界暟婵犵數鍋涢悺銊у垝瀹€鍕珘妞ゆ帒瀚烽弫鍐ㄢ攽閸屾粠鐒剧紒鈧崟顖涚厾闁诡厽甯掗崝婊勭箾閸繄鍩ｉ柡灞剧洴婵℃悂濡烽鐓庡濠电姵顔栭崰姘跺磻閵堝钃熼柛娑樼摠閸嬪嫰鏌涢幘鑼跺厡闁瑰樊浜娲川婵犲孩鐣跺┑鐐茬湴閸ㄤ粙宕洪埀? *
 * 闂傚倸鍊风欢锟犲磻閸℃稑绐楅幖杈剧岛閸嬫挾娑甸崨顔惧涧缂?70 (闂?MID=28 闂? 婵犵數鍋涢顓熸叏娴兼潙纾块柟娈垮枦婵娊鏌嶆潪鎷岊唹闁哄閰ｉ弻宥夊煛娴ｅ憡娈查梻浣斤骏閸婃妲愰幒妤佸€锋い鎺嗗亾妞ゆ洘绮撻弻鈥崇暆閳ь剟宕版惔顭戞晪闁挎繂鐗忛悿鈧梺鐟扮仢閸燁垰顭块弮鍫熲拺闁告稑锕ラ埛鎺戔攽椤旇姤灏︾€?380 (闂?MID=60 闂?闂? * - 婵犵數濮烽。浠嬪焵椤掆偓閸熷潡鍩€椤掆偓缂嶅﹪骞?MID <= 28, 闂傚倸鍊风欢锟犲磻閸℃稑绐楅幖杈剧岛閸嬫挾娑甸崨顔惧涧缂?270.
 * - 婵犵數濮烽。浠嬪焵椤掆偓閸熷潡鍩€椤掆偓缂嶅﹪骞?MID >= 60, 闂傚倸鍊风欢锟犲磻閸℃稑绐楅幖杈剧岛閸嬫挾娑甸崨顔惧涧缂?380.
 * - 闂傚倷绀侀幉锟犳偄椤掑倻涓嶉柟杈剧畱閸? 闂傚倸鍊风欢锟犲磻閸℃稑绐楅幖杈剧岛閸嬫挾鎲撮崟顐熸灆濡ょ姷鍋為崝鏍ь嚗閸曨剙绶炵€光偓閳ь剙顬婄捄琛℃斀闁绘劕寮堕ˉ鎴︽煃瑜滈崜娆撳疮閸啩鎺楀箛閻楀牏鍘告繛杈剧到閹碱偊銆傞懖鈺傚枑闁哄鐏濋弳锝団偓瑙勬礃閸旀鍒掗鐘冲仒闁炽儱鍘栨竟? * 闂傚倷鑳堕…鍫㈡崲濡も偓閵嗘帡宕烽鐘殿槸闁诲函缍嗛崰鏍箲? 婵犵數鍋犻幓顏嗙礊閳ь剚绻涙径瀣鐎?(闂佽崵鍠愮划搴㈡櫠濡ゅ啯鏆滃┑鐘插閸楁岸鏌熺紒銏犳灈缂佲偓瀹€鍕厸闁割偆鍣ラ崕顤婂┑鐘愁問閸ｎ垳寰婃ィ鍐ㄨ摕鐟滃繐鈻?^2 闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佸嚬閸欏啫顕ｆ繝姘亜濠靛倸顦▓銊╂椤愩垺澶勬慨濠傜秺濮婅棄顓兼径瀣偓鐢告煕閿旇骞栧ù鐘欏嫷娈介柣鎰级閸犳鈧娲樼换鍫ュ箖閳哄懏鍊婚柍杞版婢规洟姊虹紒妯荤叆濠⒀冮叄閹兘骞囬悧鍫㈠幈濠德板€愰崑鎾寸箾鐠囇呯暤鐎殿喗鐓℃俊鐑藉煛娴ｇ娈ゆ繝鐢靛閸愵亝鐨戦梺? *
 * @param current_mid_value 闂佽崵鍠愮划搴㈡櫠濡ゅ懎绠伴柛娑橈攻濞呯娀鏌ｅΟ铏癸紞闁崇粯姊归妵鍕箻閸楃偟浠鹃梺鎸庣〒閸犳牠寮婚敐澶婎潊闁绘ê鍚€缁敻姊洪崨濠冪叆闁瑰啿绻掗埀顒勬涧閵堟悂骞冮埡鍛闁圭粯宕归妶澶嬧拺闂傚牃鏅濈粊鐑芥煕閺傚潡顎楅柍缁樻崌楠炲鎮╅崘鑼酱缂傚倸鍊烽悞锕佹懌濠电偛鐗嗛悥濂稿蓟? * @return 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晠鎮归崶褎鈻曢柛銈嗩殕娣囧﹪濡堕崒姘闂備胶纭堕弲娑㈡儗閸岀偛绠氶柛鎰靛枛缁€瀣亜閹捐泛鏋旈柣娑栧劦濮婃椽宕崟顓夈垺绻涘顔煎籍闁糕斁鍋撳銈嗗笒閸婃悂寮告惔銊﹀€甸悷娆忓閳绘洖鈹? */
float calculate_dynamic_target_speed_quadratic(float current_mid_value) {
		#if 1
    const int MID_min_threshold = 26;   // 婵犵數鍎戠徊钘壝归崒鐐茬獥婵°倕鎳庨弸浣糕攽閸屾碍鍟為柡鍜佸墯閹便劌顫滈崱妤€顫╅梺鍝勫€戦崘锝嗩潔?28
    const int speed_at_MID_min = 230;
    const int MID_max_threshold = 30;   // 婵犵數鍎戠徊钘壝归崒鐐茬獥婵°倕鎳庨弸浣糕攽閸屾碍鍟為柡鍜佸墯閹便劌顫滈崱妤€顫╅梺鍝勫€戦崘锝嗩潔?60
    const int speed_at_MID_max = 330;

    int calculated_speed;

    if (current_mid_value <= MID_min_threshold) {
        calculated_speed = speed_at_MID_min;
    } else if (current_mid_value >= MID_max_threshold) {
        calculated_speed = speed_at_MID_max;
    } else {
        // 1. 闂佽崵鍠愮划搴㈡櫠濡ゅ啯鏆滃┑鐘插閸楁岸鏌熺紒銏犳灈缂佲偓?MID 闂傚倷鑳堕…鍫ユ晝閿曞倸鍌ㄧ憸鏂跨暦?0~1
        float mid_ratio = (current_mid_value - (float)MID_min_threshold) / 
                          ((float)MID_max_threshold - (float)MID_min_threshold);
        
        float quartic_factor = mid_ratio * mid_ratio * mid_ratio * mid_ratio; // mid_ratio^4
        
        // 3. 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晛鈹戦悩宕囶暡闁稿骸瀛╅妵鍕籍閸屾稒鐝梺鎼炲妼閵堢顫忓ú顏嶆晝闁靛牆鎳忛悗濠氭煟?
        float speed_range = (float)(speed_at_MID_max - speed_at_MID_min);
        calculated_speed = (int)((float)speed_at_MID_min + quartic_factor * speed_range);
    }

    return calculated_speed;
		
		#endif
		
		
}

/*********************rgb*********************//*********************rgb*********************//*********************rgb*********************/	



//void set_rgb_pins(int p26_val, int p74_val, int p07_val) {
//    P26 = p26_val;
//    P74 = p74_val;
//    P07 = p07_val;
//}

//void control_rgb_led_conditional(float check_value, float abs_threshold, RgbColorCode_t feedback_color, int enable_state) {
//    // 濠电姷顣藉Σ鍛村磻閳ь剟鏌涚€ｎ偅宕岄柡宀嬬磿娴狅妇鎷犻幓鎺戭潚闂佽崵鍋炴穱鐑樸仈閸濄儳鐭欏鑸靛姦閺佸倿鏌涘☉鍗炴灕闁瑰鍋呯换娑氣偓鐢殿焾闉嬪銈嗘⒐閸旀危?
//    // 1. 闂傚倷鑳堕崕鐢稿疾濠婂牆鍨傞柛鎾茬劍閺嗘粓鏌涢锝嗙缁炬儳缍婇弻銈吤圭€ｎ偅鐝曢梺浼欑秬娴滎剛妲愰幘瀛樺闁革富鍘鹃悾闈涱渻閵堝棙澶勯柛锝忕到閻ｇ兘骞囬弶璺槯闂佺绻愰妵娆撴晝閸屾稓鍘卞┑顔矫晶浠嬫偩閹惰姤鐓欐繛鑼额唺闁垳鈧娲樺畝鎼佸箖瑜斿畷鐓庘攽閸粎鐜婚梻浣哄劦閸撴繄鏁幒妤€绠犻柟鎹愬煐瀹曟煡鏌涢幇闈涙灍闁抽攱鎹囬弻娑㈩敃閿濆洤顩梺?//    // 2. RGB LED 闂傚倷绀侀幖顐も偓姘卞厴瀹曡瀵奸弶鎴犵暰婵炶揪绲块崕銈夊磻閺嶃劊浜滈柟鎵虫櫅閻忣亪鏌涙惔銏㈡噰闁?(enable_state == 1)
//	
//		#if 1
//    if ((fabs(check_value) >= abs_threshold) && (enable_state == 1)) {
//        switch (feedback_color) {
//            case RGB_COLOR_OFF:
//                set_rgb_pins(0, 0, 0);
//                break;
//            case RGB_COLOR_WHITE:
//                set_rgb_pins(0, 0, 1);
//                break;
//            case RGB_COLOR_CYAN:
//                set_rgb_pins(0, 1, 0);
//                break;
//            case RGB_COLOR_YELLOW_GREEN:
//                set_rgb_pins(1, 0, 0);
//                break;
//            case RGB_COLOR_MAGENTA:
//                set_rgb_pins(0, 1, 1);
//                break;
//            case RGB_COLOR_GREEN:
//                set_rgb_pins(1, 1, 0);
//                break;
//            case RGB_COLOR_RED:
//                set_rgb_pins(1, 0, 1);
//                break;
//            case RGB_COLOR_BLUE:
//                set_rgb_pins(1, 1, 1);
//                break;
//            default:
//                set_rgb_pins(0, 0, 0); // 闂備浇顕уù鐑藉箠閹惧嚢鍥敍閻愯尙鐓戦棅顐㈡处濞叉浜告惔銊︾厽闁靛牆楠搁悘杈ㄧ箾閻撳海鐒搁柡灞诲妼閳藉螣閸噮浼冮梻浣告啞椤忔悂宕堕妸銏″闂備浇宕垫慨鏉戔枖婵?
//                break;
//        }
//    } else {
//        set_rgb_pins(0, 0, 0);
//    }
//		#endif
//		

//		
//}

//void control_rgb_led( RgbColorCode_t feedback_color) {

//        switch (feedback_color) {
//            case RGB_COLOR_OFF:
//                set_rgb_pins(0, 0, 0);
//                break;
//            case RGB_COLOR_WHITE:
//                set_rgb_pins(0, 0, 1);
//                break;
//            case RGB_COLOR_CYAN:
//                set_rgb_pins(0, 1, 0);
//                break;
//            case RGB_COLOR_YELLOW_GREEN:
//                set_rgb_pins(1, 0, 0);
//                break;
//            case RGB_COLOR_MAGENTA:
//                set_rgb_pins(0, 1, 1);
//                break;
//            case RGB_COLOR_GREEN:
//                set_rgb_pins(1, 1, 0);
//                break;
//            case RGB_COLOR_RED:
//                set_rgb_pins(1, 0, 1);
//                break;
//            case RGB_COLOR_BLUE:
//                set_rgb_pins(1, 1, 1);
//                break;
//            default:
//                set_rgb_pins(0, 0, 0); // 闂備浇顕уù鐑藉箠閹惧嚢鍥敍閻愯尙鐓戦棅顐㈡处濞叉浜告惔銊︾厽闁靛牆楠搁悘杈ㄧ箾閻撳海鐒搁柡灞诲妼閳藉螣閸噮浼冮梻浣告啞椤忔悂宕堕妸銏″闂備浇宕垫慨鏉戔枖婵?
//                break;
//        }

//}

/*********************闂備浇宕甸崰鎰版偡閿旂偓鏆滈柟鐑樻煛閸嬫挾鎲撮崟顐熸灆闂?********************//*********************闂備浇宕甸崰鎰版偡閿旂偓鏆滈柟鐑樻煛閸嬫挾鎲撮崟顐熸灆闂?********************//*********************闂備浇宕甸崰鎰版偡閿旂偓鏆滈柟鐑樻煛閸嬫挾鎲撮崟顐熸灆闂?********************/	
/*******************************闂傚倷绀侀幉锟犲Φ濞戙垹鐒垫い鎺嗗亾缁剧虎鍘艰婵犲﹤鐗婇悡娑㈡煕閵夋垵鍟崐顖炴⒑绾懐鍫柛濠冩礋閸?*******************************//*******************************闂傚倷绀侀幉锟犲Φ濞戙垹鐒垫い鎺嗗亾缁剧虎鍘艰婵犲﹤鐗婇悡娑㈡煕閵夋垵鍟崐顖炴⒑绾懐鍫柛濠冩礋閸?*******************************//*******************************闂傚倷绀侀幉锟犲Φ濞戙垹鐒垫い鎺嗗亾缁剧虎鍘艰婵犲﹤鐗婇悡娑㈡煕閵夋垵鍟崐顖炴⒑绾懐鍫柛濠冩礋閸?*******************************/

///* 闂佽娴烽弫濠氬磻婵犲洤绐楅柡宥庡幖閸ㄥ倿骞栧ǎ顒€濡肩紒顐㈢Ч閺屾盯顢曢敐鍡欘槰闂?*/
//float dt_my = 0.005f;
//float rad2deg = 57.29578f;  /* 闂佽瀛╅鏍疮椤愶箑绀冩い顓熷灥閺佷粙姊绘担鍛婂暈閻㈩垱顨嗛崚濠冪鐎ｅ墎绋忓┑掳鍊撻悞锕傚磿閻斿吋鐓ラ柡鍐ㄥ€告禍楣冩煟閿濆鎲鹃柡宀€鍠栭悰顕€宕归鍙ョ磿闂備椒绱紞鈧€规洜鏁诲畷鐘诲冀椤愩倗锛滃┑鈽嗗灣缁垶顢?*/

///* 闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柛宀€鍎戠紞鏍煙閻楀牊绶茬痪鎯ь煼閻擃偊宕堕妸锕傛毐闂佸啿鎼幊蹇涙偂?*/
//float origin_ax_offset = 0;
//float origin_ay_offset = 0;
//float origin_az_offset = 0;
//float origin_gx_offset = 0;
//float origin_gy_offset = 0;
//float origin_gz_offset = 0;

///* mpu闂傚倷鑳堕～瀣礋椤愩埄娼旈梻浣虹帛閻楊厾寰婇崸妤€绀岄柡宥庡亞绾惧吋淇婇姘儓缂併劊鍎崇槐鎾存媴閸濆嫅锝嗐亜閵娿儲鍤囩€规洝娅ｉ埀顒勬涧閻楁捇寮诲☉姗嗙叆闁告劦浜濋～婵嬫⒑閸濆嫷妲洪柛瀣枔閹广垹鈹戦崼銏㈢暥闂佺鐬奸崑鐐哄磿瀹ュ鐓曢柡鍥ュ妼閻忔瑩鏌涚€ｎ偅宕岄柟顔哄灩閻ｏ繝鏌囬敃鈧弫?*/
//float roll_v = 0;
//float pitch_v = 0;
//float yaw_v = 0;

///* 婵犵數鍋為崹鍫曞箰閹间絸鍥敋閳ь剟寮荤仦绛嬬叆闁割偆鍠庢禍褰掓⒑閻撳孩鎲告い锝勭矙瀹?*/
//float gyro_roll_my = 0;
//float gyro_pitch_my = 0;
//float gyro_yaw = 0;

///* 闂備浇宕甸崰鎰版偡閵夆晛纾归柛锔诲幘閻牓鏌熺紒銏犳灍闁稿鍔欓弻銈夊传閵夘喗姣岄梺绋款儐閹稿骞忛崨顖氬闁哄洨鍋樻竟?*/
//float acc_roll = 0;
//float acc_pitch = 0;

///* 闂傚倷绀侀幖顐︽偋閸愵喖纾婚柟鍓х帛閻撴洘鎱ㄥ鍡楀闁圭櫢绲剧换娑㈠醇閻旈浠搁柣搴ㄦ涧閵堟悂鐛箛鎾舵殕濠电姴鍊归崐搴ㄦ⒒?*/
//float k_roll = 0;
//float k_pitch = 0;
//float k_yaw = 0;

///* 闂備浇宕垫慨鏉懨归崒鐐插瀭闁哄顕抽悢鍝ョ瘈闁搞儜鍜佹敤闂傚鍋勫ú锕€顫忚ぐ鎺撳亗婵炲棙鍨熼崑鎾绘偡閺夋妫岄梺鍝ュУ瀹€鎼佸箖閿熺姴鐓涢柛娑卞枓閹糕偓?*/
//float e_P[2][2];
///* 闂傚倷绀侀幉锟犲Φ濞戙垹鐒垫い鎺嗗亾缁剧虎鍘艰婵犲﹤鐗婇悡娑㈡煕閵夈垺娅呴悽顖濆煐閵囧嫰濡堕崨顓熸闂佽鍨扮€氼厾鎹㈠┑瀣妞ゆ牗鑹剧敮锝夋⒒?*/
//float k_k[2][2];

//void origin_data() {
//    int i;

//    origin_ax_offset = 0;
//    origin_ay_offset = 0;
//    origin_az_offset = 0;
//    origin_gx_offset = 0;
//    origin_gy_offset = 0;
//    origin_gz_offset = 0;

//    for (i = 0; i < 300; i++) {
//        imu660ra_get_acc();
//        imu660ra_get_gyro();

//        origin_ax_offset += imu660ra_acc_transition(imu660ra_acc_x);
//        origin_ay_offset += imu660ra_acc_transition(imu660ra_acc_y);
//        origin_az_offset += imu660ra_acc_transition(imu660ra_acc_z);

//        origin_gx_offset += imu660ra_gyro_transition(imu660ra_gyro_x);
//        origin_gy_offset += imu660ra_gyro_transition(imu660ra_gyro_y);
//        origin_gz_offset += imu660ra_gyro_transition(imu660ra_gyro_z);
//    }

//    origin_ax_offset /= 300.0f;
//    origin_ay_offset /= 300.0f;
//    origin_az_offset /= 300.0f;
//    origin_gx_offset /= 300.0f;
//    origin_gy_offset /= 300.0f;
//    origin_gz_offset /= 300.0f;
//}

//void kalanma_data() {
//    float ax, ay, az;
//    float gx, gy, gz;
//    float sin_roll, cos_roll, sin_pitch, cos_pitch;
//    float acc_ax_adj, acc_ay_adj, acc_az_adj;

//    imu660ra_get_acc();
//    imu660ra_get_gyro();

//    ax = imu660ra_acc_transition(imu660ra_acc_x);
//    ay = imu660ra_acc_transition(imu660ra_acc_y);
//    az = imu660ra_acc_transition(imu660ra_acc_z);

//    gx = imu660ra_gyro_transition(imu660ra_gyro_x);
//    gy = imu660ra_gyro_transition(imu660ra_gyro_y);
//    gz = imu660ra_gyro_transition(imu660ra_gyro_z);

//    sin_roll = (float)sin(k_roll / rad2deg);
//    cos_roll = (float)cos(k_roll / rad2deg);
//    sin_pitch = (float)sin(k_pitch / rad2deg);
//    cos_pitch = (float)cos(k_pitch / rad2deg);

//    roll_v = (gx - origin_gx_offset)
//           + (sin_pitch * sin_roll / cos_pitch) * (gy - origin_gy_offset)
//           + (sin_pitch * cos_roll / cos_pitch) * (gz - origin_gz_offset);
//    pitch_v = cos_roll * (gy - origin_gy_offset) - sin_roll * (gz - origin_gz_offset);
//    yaw_v = (sin_roll / cos_pitch) * (gy - origin_gy_offset)
//          + (cos_roll / cos_pitch) * (gz - origin_gz_offset);

//    roll_v /= 100.0f;
//    pitch_v /= 100.0f;
//    yaw_v /= 100.0f;

//    gyro_roll_my = k_roll + dt_my * roll_v;
//    gyro_pitch_my = k_pitch + dt_my * pitch_v;
//    gyro_yaw = k_yaw / 30.0f + dt_my * (yaw_v + 0.2f);  /* 闂傚倷鑳堕…鍫ヮ敄閸℃鈹嶆繛宸悍缂嶆牗淇婇妶鍕妽闁稿海鍠栭弻銊╁即濮樿精绠炴繝銏ｅ煐閸旀洜澹?*/

//    /* Step 2: 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晛鈹戦悩瀹犲缂佲偓閸岀偞鐓曟い鎰Т閸斻倗鈧娲╃紞渚€骞冭ぐ鎺戠倞閻犻缚娅ｆ禒鈺佲攽閳藉棗鐏犻柨鏇ㄤ簻閻ｇ兘妾辩紒鐘崇洴瀵噣鍩€椤掑嫭鍋傛繛鍡樺灍閸嬫捇鎮烽弶娆炬闂佸摜濮靛畝鎼佸箖閿熺姴鐓涢柛娑卞枓閹糕偓?*/
//    e_P[0][0] += 0.0025f;
//    e_P[1][1] += 0.0025f;

//    /* Step 3: 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶鎼愮痪顓涘亾闂備胶鍋ㄩ崕杈╁椤撶倫鎺戭煥閸喓鍘搁梺鎼炲劗閺呮稓鏁懜鐐逛簻闁靛牆鎳庨弳锝夋煙椤栨艾顏紒?*/
//    k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3f);
//    k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3f);

//    /* Step 4: 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晠鏌ゆ慨鎰偓鏍窗閸℃稒鐓曢悘鐐插⒔閵嗘帡鏌曢崼鐔稿唉闁哄矉绲借灒闁割煈鍠氶崢顐︽⒑?*/
//    acc_ax_adj = ax - origin_ax_offset;
//    acc_ay_adj = ay - origin_ay_offset;
//    acc_az_adj = az - origin_az_offset;

//    acc_roll = (float)(atan2(acc_ay_adj, acc_az_adj) * rad2deg);
//    acc_pitch = (float)(-atan2(acc_ax_adj,
//        sqrt(acc_ay_adj * acc_ay_adj + acc_az_adj * acc_az_adj)) * rad2deg);

//    k_roll = gyro_roll_my + k_k[0][0] * (acc_roll - gyro_roll_my);
//    k_pitch = gyro_pitch_my + k_k[1][1] * (acc_pitch - gyro_pitch_my);
//    k_yaw = 30.0f * gyro_yaw;

//    /* Step 5: 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶鎼愮痪顓涘亾闂傚鍋勫ú锕€顫忚ぐ鎺撳亗婵炲棙鍨熼崑鎾绘偡閺夋妫岄梺鍝ュУ瀹€鎼佸箖閿熺姴鐓涢柛娑卞枓閹糕偓?*/
//    e_P[0][0] *= (1.0f - k_k[0][0]);
//    e_P[1][1] *= (1.0f - k_k[1][1]);
//}

///* 闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柛宀€鍎戠紞鏍煙閻楀牊绶茬紒鈧畝鍕厸鐎广儱楠告禍鐐烘煕濞嗘劕濮嶆慨濠傤煼瀹曟﹢顢欓懞銉╂暘闁荤喐绮嶅姗€藝閻㈢钃熺€光偓閸愵亞鏉搁梺闈涳紡閸滀焦袩闂傚倸顭崑鍕洪妶澶婄疇婵せ鍋撳┑锛勵棎缁犳稑鈽夊Ο纰辨敤闂備礁鎼ú銊︽叏閹绢噮鏁傛い蹇撶墛閻撴洟鏌熼柇锕€骞橀柟铏礈缁?*/
//void init_kalman() {
//    e_P[0][0] = 1.0f; e_P[0][1] = 0.0f;
//    e_P[1][0] = 0.0f; e_P[1][1] = 1.0f;

//    k_k[0][0] = 0.0f; k_k[0][1] = 0.0f;
//    k_k[1][0] = 0.0f; k_k[1][1] = 0.0f;
//}






/**
 * @brief 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺?encoder_speedup_sign 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑?encoder_speedup_element 闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佸嚬閸撶喖宕洪埀顒併亜閹烘垵浜為柛搴ｅ█閺? * 婵犵數濮烽。浠嬪焵椤掆偓閸熷潡鍩€椤掆偓缂嶅﹪骞?sign 婵?1, 闂傚倷绀侀幉锛勬暜濡ゅ懏鍋￠柕澶嗘櫆閸嬪嫰鏌嶈閸撴瑩婀佸┑鐘诧工閸熸挳顢撳Δ鍛厱婵炲棗绻掔粻鍐裁归悪鍛暤闁糕斁鍋撳銈嗗笒鐎氼參宕?l_speed_now 闂?r_speed_now 闂備礁鎼ˇ顐﹀疾濠婂懐鐭欓柡宥庡幑閳ь兛绶氶獮瀣偐閹颁焦缍楁俊鐐€栭悧妤呮嚌妤ｅ喛缍栭柨婵嗩槹閻? * 闂傚倷绀侀幉锟犳偄椤掑倻涓嶉柟杈剧畱閸? 闂?element 濠电姷鏁搁崑鐐哄箰閹间礁绠犳俊顖氥偨瑜版帒纾奸柣鎰棘? * * @param p_encoder_speedup_element 闂傚倷绀佸﹢閬嶁€﹂崼婢濇椽濡舵径濠勭暫濠电姴锕ら幊蹇涘疮閸濆娊褰掓晲閸偅缍堢紓浣鸿檸閸ㄥ爼寮婚敓鐘茬＜婵炴垶锕╁Λ鍡椻攽?encoder_speedup_element 闂傚倷绀侀幉锟犳偡閿曞倹鏅濋柕蹇嬪€曢梻顖涚箾瀹割喕绨奸柛搴＄Ч閺屾盯寮撮妸銉ヮ潻闂佷紮绠戠紞濠傤潖濞差亝瀵犲璺鸿嫰椤ｅ爼姊? * @param current_encoder_speedup_sign 闂佽崵鍠愮划搴㈡櫠濡ゅ懎绠伴柛娑橈攻濞呯娀鏌ｅΟ鑲╁笡闁?encoder_speedup_sign 闂傚倷绀侀幖顐ょ矓閺夋嚚娲Χ閸ャ劌搴婇梺鍝勫暊閸嬫捇鏌熷畡鐗堝櫣妞ゆ挸銈稿畷姗€鏁愰崨顒€顥? */
void update_encoder_speedup_value(float* p_encoder_speedup_element, 
                                  int current_encoder_speedup_sign) 
{
    if (current_encoder_speedup_sign == 1) {

        *p_encoder_speedup_element += (fabs(l_speed_now) + fabs(r_speed_now)) * 0.5f * 0.00003895f;
    } else {
        *p_encoder_speedup_element = 0.0f;
    }
}


void update_gyro_angle_accumulator(float* p_angle_accumulator,
                                   int sign_flag)
{
    if (sign_flag == 1) {
        *p_angle_accumulator += gyro_data[0] * 0.005f;
    } else {
        *p_angle_accumulator = 0.0f;
    }
}




