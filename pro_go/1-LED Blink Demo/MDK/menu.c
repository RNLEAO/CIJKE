#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "isr.h"

uint8 key_mode_local =0;
/****************闂傚倸鍊烽悞锕傛儑瑜版帒鍨傜憸鐗堝笚閸婅埖鎱ㄥ鍡楀幋闁衡偓閿曞倹鐓?***************//****************闂傚倸鍊烽悞锕傛儑瑜版帒鍨傜憸鐗堝笚閸婅埖鎱ㄥ鍡楀幋闁衡偓閿曞倹鐓?***************//****************闂傚倸鍊烽悞锕傛儑瑜版帒鍨傜憸鐗堝笚閸婅埖鎱ㄥ鍡楀幋闁衡偓閿曞倹鐓?***************/
void display_motor(_PID* sptr, float speed_now, uint16 pwm_duty, uint8 key_press, uint8 dir_id)
{
    static unsigned char key_mode_local = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode_local++;
        if (key_mode_local >= 3) key_mode_local = 0;
    }

    switch (key_mode_local)
    {
        case 0:
            adjust_parameter_by_key_float(key_press, &sptr->kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_press, &sptr->ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_press, &sptr->kd, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    if (dir_id == 0)
        lcd_showstr(0, 0, "left");
    else
        lcd_showstr(0, 0, "right");

    lcd_showfloat(0, 1, sptr->kp, 3, 6);
    lcd_showstr(80, 1, " Kp ");
    lcd_showstr(120, 1, (key_mode_local == 0) ? "<" : " ");

    lcd_showfloat(0, 2, sptr->ki, 3, 6);
    lcd_showstr(80, 2, " Ki ");
    lcd_showstr(120, 2, (key_mode_local == 1) ? "<" : " ");

    lcd_showfloat(0, 3, sptr->kd, 3, 6);
    lcd_showstr(80, 3, " Kd ");
    lcd_showstr(120, 3, (key_mode_local == 2) ? "<" : " ");

    lcd_showfloat(0, 4, sptr->kp_out, 3, 2);
    lcd_showstr(80, 4, " Pout ");

    lcd_showfloat(0, 5, sptr->ki_out, 3, 2);
    lcd_showstr(80, 5, " Iout ");

    lcd_showuint16(0, 6, pwm_duty);
    lcd_showstr(80, 6, " PWM ");

    lcd_showfloat(0, 7, speed_now, 3, 3);
    lcd_showstr(80, 7, " V_now ");

    lcd_showuint16(0, 8, sptr->Target);
    lcd_showstr(80, 8, " Tar ");
}

void display_g(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 6) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kd, (float)x_t_int * x_t_float);
            break;
        case 3:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kp1, (float)x_t_int * x_t_float * 0.01f);
            break;
        case 4:
            adjust_parameter_by_key_float(key_value_test, &Gyro_PID.kd2, (float)x_t_int * x_t_float);
            break;
        case 5:
            adjust_parameter_by_key_float(key_value_test, &err_t, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "gyro");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, Gyro_PID.kp, 3, 3);
    lcd_showstr(89, 1, " Kp ");
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 2, Gyro_PID.ki, 4, 3);
    lcd_showstr(89, 2, " Ki ");
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 3, Gyro_PID.kd, 3, 3);
    lcd_showstr(89, 3, " Kd ");
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showfloat(0, 4, Gyro_PID.kp1, 3, 4);
    lcd_showstr(89, 4, " Kp1 ");
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showfloat(0, 5, Gyro_PID.kd2, 3, 4);
    lcd_showstr(89, 5, " Kd2 ");
    lcd_showstr(120, 5, key_mode == 4 ? "<" : " ");

    lcd_showfloat(0, 6, err_t, 3, 3);
    lcd_showstr(89, 6, " et ");
    lcd_showstr(120, 6, key_mode == 5 ? "<" : " ");

    lcd_showfloat(0, 7, Gyro_PID.out, 5, 2);
    lcd_showstr(89, 7, " out ");

    lcd_showfloat(0, 8, Gyro_PID.err, 5, 2);
    lcd_showstr(89, 8, " err ");
}

void display_t(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kp, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.ki, (float)x_t_int * x_t_float);
            break;
        case 2:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kd, (float)x_t_int * x_t_float);
            break;
        case 3:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kp1, (float)x_t_int * x_t_float * 0.01f);
            break;
        case 4:
            adjust_parameter_by_key_float(key_value_test, &Turn_PID.kd2, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "turn");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, Turn_PID.kp, 3, 3);
    lcd_showstr(89, 1, " Kp ");
    lcd_showstr(120, 1, (key_mode == 0) ? "<" : " ");

    lcd_showfloat(0, 2, Turn_PID.ki, 4, 3);
    lcd_showstr(89, 2, " Ki ");
    lcd_showstr(120, 2, (key_mode == 1) ? "<" : " ");

    lcd_showfloat(0, 3, Turn_PID.kd, 3, 3);
    lcd_showstr(89, 3, " Kd ");
    lcd_showstr(120, 3, (key_mode == 2) ? "<" : " ");

    lcd_showfloat(0, 4, Turn_PID.kp1, 3, 4);
    lcd_showstr(89, 4, " Kp1 ");
    lcd_showstr(120, 4, (key_mode == 3) ? "<" : " ");

    lcd_showfloat(0, 5, Turn_PID.kd2, 3, 3);
    lcd_showstr(89, 5, " Kd2 ");
    lcd_showstr(120, 5, (key_mode == 4) ? "<" : " ");

    lcd_showfloat(0, 6, Turn_PID.out, 5, 2);
    lcd_showstr(89, 6, " out ");

    lcd_showfloat(0, 7, Turn_PID.err, 3, 5);
    lcd_showstr(89, 7, " err ");

    lcd_showfloat(0, 8,
        err_H * (L - R) +
        err_X * (LM - RM), 3, 4);

    lcd_showfloat(60, 8,
        err_HM * (L + R) +
        err_D * fabs(LM - RM) +
        err_M * MID, 3, 4);
}







//void display_straight_param(void)
//{
//    static unsigned char key_mode = 0;
//    unsigned char key_value_test;

//    key_value_test = key_scan(1);

//    // 闂傚倸鍊风粈渚€骞夐敍鍕殰闁圭儤鍤氬ú顏呮櫇闁逞屽墴閹箖鎮滈懞銉ユ濡炪倖甯掗崐濠氭晬濠靛鈷戦柛婵嗗閳诲鏌涢幘瀵搞€掗柡渚囧櫍瀹曞崬螖婵犲啫鐦滈梻渚€娼ч悧鍡椕洪敃鍌ゆ晜闁绘棁娅ｇ壕濂稿级閸稑濡块柟鍐叉嚇閺岀喐顦版惔鈾€鏋呭銈冨灪缁嬫垿锝炲┑瀣垫晣婵炴垶鐟ф禍鐐烘⒒閸屾艾鈧悂宕愰幖浣哥柈妞ゆ劧闄勯崑瀣煕閳╁啰鈽夐柣?/ 缂傚倸鍊搁崐椋庣矆娓氣偓钘濆ù鐘差儏閸ㄥ倸霉閸忓吋缍戦柛銊ュ€块獮鏍庨鈧俊浠嬫煃闁垮宕岄柡灞剧洴椤㈡洟鏁愰崶褜娲梻浣侯焾椤戝棝骞愰幆褉妲堥柛顭戝亽濡插綊骞栨潏鍓хУ婵?
//    if (key_value_test == 3) {
//        lcd_clear(WHITE);
//        key_mode++;
//        if (key_mode >= 2) key_mode = 0; // 闂傚倸鍊风粈渚€骞夐敓鐘冲仭妞ゆ牗绋撻々鍙夌節闂堟稒锛旈柤鏉跨仢闇夐柨婵嗘噺鐠愶繝鏌嶇紒妯活棃闁哄本娲熷畷鐓庘攽閸♀晙绱旈梻浣规偠閸婃洟顢栨径鎰摕婵炴垶菤閺嬪酣鐓崶銊﹀皑闁稿鎹侀妵鎰板箳閹寸偠绶?//    }

//    // 闂傚倸鍊风粈渚€骞夐敓鐘冲仭闁靛鏅涚壕鍦喐閻楀牆绗掓慨瑙勭叀閺岋綁寮崒姘闁诲孩鍑归崜鐔煎蓟濞戙垹绠涢梻鍫熺☉缁犲綊鏌?
//    switch (key_mode) {
//        case 0:
//            adjust_parameter_by_key_float(key_value_test, &straight_err_threshold, (float)x_t_int * x_t_float);
//            break;
//        case 1:
//            adjust_parameter_by_key_float(key_value_test, &straight_integral_threshold, (float)x_t_int * x_t_float);
//            break;
//        default:
//            break;
//    }

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼幐搴ｃ€愰梺鍛婂姀閸嬫捇姊绘担瑙勫仩闁稿孩妞藉畷婊堟晝閸屾碍杈?
//    lcd_showstr(0, 0, "straight");
//    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼崸妤€鎽甸柣搴㈢绾板秹濡甸崟顖氬唨闁靛ě灞炬闂備礁纾划顖毭洪悢濂夋綎濠电姵鑹剧粈鍐煏婵炑冨枤閺嗩偊姊?//    lcd_showfloat(0, 1, straight_err_threshold, 2, 3);
//    lcd_showstr(60, 1, " Err_Th ");
//    if (key_mode == 0) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熸径绋挎儓闂佹椿鍘介幐濠氬Φ閸曨垰鍐€闁靛鍔岄ˉ婵嬫⒑鐠囨煡顎楁い顓炲槻椤繑绻濆顒傤槰闂侀潧臎閸愮偓姣囬梻?//    lcd_showfloat(0, 2, straight_integral_threshold, 2, 3);
//    lcd_showstr(60, 2, " Int_Th ");
//    if (key_mode == 1) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");
//		
//	 lcd_showfloat(0, 3, gyro_right_angle, 2, 3);
//   lcd_showstr(60, 3, " rtang ");

//		

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼悡搴濈紦婵炲瓨绮岀紞濠囧蓟閻旂厧绠氱憸宥夊汲鏉堛劊浜滈柕鍫濇噺閸ゅ洭鏌熼鑲╃Ш鐎规洖澧庨幑鍕惞闁稑鎽嬪┑鐘殿暯濡插懘宕规导瀛樻櫇妞ゅ繐瀚弳锕傛煙闁箑鏋熼柛蹇旂矊椤啰鈧綆浜濋幑锝嗐亜閿斿搫绲緐rn_PID.err闂?//    lcd_showfloat(0, 4, Turn_PID.err, 3, 3);
//    lcd_showstr(89, 4, " Err ");

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼悡搴濈紦婵炲瓨绮岀紞濠囧蓟閻旂厧绠氱憸宥夊汲鏉堛劊浜滈柕鍫濇噺閸ｇ晫绱掓潏銊ョ瑲鐎垫澘瀚灒闁惧繘鈧稑鎽嬮梻鍌欑劍閹爼宕濆畝鍕厐闁挎繂鎳愰弳?//    lcd_showfloat(0, 5, encoder_straight_element, 3, 3);
//    lcd_showstr(89, 5, " Int ");

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼崹顔惧帿闂佺顑嗛崝娆撳蓟閵堝浼犻柕澶樺枟濮ｅ矂姊虹粙娆惧剱闁圭顭烽獮蹇涘川閺夋垶宓嶅銈嗘尵閸嬬喐瀵奸崶顒佲拻濠电姴楠告禍婊勭箾鐠囇呯暤濠碉紕鏁婚幃娆撴偨閻㈡妲?//    lcd_showstr(0, 7, "Flag:");
//    if (right_angle_flag) lcd_showstr(50, 7, "1"); else lcd_showstr(50, 7, "0");
//}


// 闂傚倷娴囬褍顫濋敃鍌︾稏濠㈣埖鍔栭崕妤併亜閺傚灝鈷斿☉鎾崇Ч閺岋綁寮崹顔藉€梺绋块濞寸兘鎯€椤忓牆绠氱憸婊堝磿濞戞瑧绠剧痪顓㈩棑缁♀偓闂佸搫鏈惄顖炵嵁閹烘柡鍋撻敐搴濈按闁稿鎹囧浠嬵敇閻愯埖鎲伴梻浣规偠閸庮垶宕濇惔銊ョ厺闁哄啠鍋撻柕鍥у楠炴鈧潧鎽滈悾鐢告⒑鐎圭媭鍤欑紒缁橈耿瀵鏁愭径娑氱◤濡炪倖宸婚崑鎾斥攽閳╁啫顕滅紒缁樼洴瀹曞ジ鎮㈡搴濈礉闂備焦鎮堕崝宥夊磿閹惰В鈧箓濡搁埡浣侯槹濡炪倖鎸鹃崑鐔稿閸ヮ剚鈷掑ù锝呮啞閸熺偤鏌涢妸鈺傛锭閾荤偞淇婇妶鍛櫣闁绘帒鐏氶妵鍕箳閸℃ぞ澹曢梻浣圭湽閸斿酣宕楀鈧畷娲焵?#define RIGHT_ANGLE_TARGET_ANGLE    76.0f   // 闂傚倸鍊风粈渚€骞夐敍鍕殰闁搞儺鍓欑壕褰掓煛瀹ュ骸骞栭柦鍐枛閺屾盯濡烽敐鍛瀴婵犳鍠楃划宥夊Φ閸曨垰绠抽柛鈩冦仦婢规洟姊婚崒姘偓绋课涘Δ鈧灋婵炴垯鍨归悞鍨亜閹烘垵鈧綊宕甸埀顒勬⒑閸濆嫮澧遍柛鎾跺枎閻ｇ兘鎮烽柇锔藉兊闂佸吋鎮傚褔宕滈銏♀拺闁告稑锕ユ径鍕煕濡吋娅曠紒顕嗙到铻栭柛娑卞枤閸樻悂鎮楅獮鍨姎闁瑰啿绻掔划姘跺箰鎼存稐绨诲銈嗗姧缂嶅棗鈻撻弴銏＄厵?#define RIGHT_ANGLE_EXIT_COUNT      40      // 闂傚倷绀侀幖顐λ囬柆宥呯？闁圭増婢樼粈鍫熺箾閸℃ɑ灏伴柍閿嬬墵閺屾盯鍩勯崘顏呭枦闂佺顑嗛幑鍥箖閻戣姤鍋嬮柛顐ゅ枑鐎氭娊姊绘担瑙勫仩闁告柨绉撮悾婵堢矙鐠恒劍娈鹃梺闈涱檧闂勫嫰宕曢悢鎼炰簻闁哄秲鍔庨惌灞俱亜閿旇娅嶉柟顔筋殜閹瑩鎮烽幍鍐蹭壕婵炴垯鍨归弰銉╂煛瀹擃喖鏈€靛矂姊虹粙璺ㄧ伇闁稿鍋ら幃鐐寸節濮橆厾鍘撻悷婊勭矒瀹曟粓鎮㈡搴㈡濠电偛妫欓幐濠氬箚閻愮儤鍋℃繛鍡楃箰椤忣偊鏌￠崱娆忊枅闁哄矉缍侀幃鈺傛綇椤愵偄鏁奸梻浣告啞閹歌崵鎹㈤崼婵愬殨闁规儼濮ら崑鍕煟閹捐櫕鎹ｉ柨?// ==========================================================
// 闂傚倸鍊风粈渚€骞夐敓鐘插瀭闁稿繐鍚嬮崣蹇涙煏閸繍妲告慨瑙勭叀閺岋綁寮崒姘粯闁荤喐鐟辨俊鍥焵椤掆偓缁犲秹宕曟潏鈺傚床闁圭儤姊婚悳缁樻叏婵犲啯鏁痵play_right_angle_param
// 闂傚倸鍊风粈渚€骞夐垾鎰佹綎濠电姵鑹剧粣妤呮煛閸ャ儱鐏╃紒鈧崒娑欏弿婵＄偠顕ф禍楣冩⒑? 闂傚倸鍊风欢姘焽婵犳碍鍤堢憸鐗堝笒閻ゎ噣鏌熺拠鎻掝嚛濠电姷鏁搁崑鐐哄垂閸洖绠伴柟闂寸劍閺呮繈鏌曟径鍫濆壔闁绘梻顭堢欢鐐碘偓鍏夊亾闁逞屽墴閹矂宕卞☉娆戝弰闂婎偄娲﹂崙鐟邦焽閹邦喚纾肩紓浣靛灩閻忥箓鏌熼鐓庢Щ闁宠閰ｉ獮鍥敆閸屻倕鍓┑鐘垫暩閸嬬偛顭囧▎寰稑螖閸滀焦鏅梺鎸庣箓椤︿粙寮鍡欑瘈濠电姴鍊搁顐︽煛閸℃瑥鈻堟慨濠呮缁瑧鎹勯妸褜鍞瑰┑鐘垫暩閸嬫盯鏁冮鍕靛殨妞ゆ劧瀵岄弫鍥煏韫囨洖啸闁绘挻鍨甸埞鎴︻敊婵劒绮堕梺绋款儐閹瑰洭寮诲☉銏℃櫜闁糕剝菧娴犮垺绻濈喊妯峰亾瀹曞洦鎲奸梺鐟板槻閹虫ê鐣烽锕€绀嬮柟鎼灣閻ｇ敻鏌″畝鈧崰鏍€侀弽顓炵閹兼惌鍠楅埢濠傗攽閻愯尙鎽犵紒顔肩Ф閺侇噣鍨惧畷鍥ㄦ闂佹寧绻傚Λ娑氬姬閳ь剟姊虹粙鑳潶闁稿﹥娲栭埢鎾诲箹娴ｅ湱鍘甸梺缁樻尭濞寸兘骞楅悩铏弿濠电姴鎷嬮崵娆撴煃缂佹ɑ宕屾鐐差儔閺佸啴鍩€椤掑倻鐭嗛柛鈩冪⊕閻撳繘鏌涢锝囩畺妞ゃ儳鍋ら弻锝夊箳瀹ュ洨鐓撻梺鍝勫閳ь剚鍓氶崥瀣煕濞戝崬鏋熼柟顔界懇濮婃椽妫冮埡鍐ｉ梺鎼炲妼閻栫厧顕ｉ銏╁悑濠㈣泛锕崬璺衡攽閻樿宸ラ柛鐔哄█瀵瓕绠涘☉娆屾嫼濠电偠灏欑划顖涚箾閸ヮ剚鐓曢柡鍌涘閹癸絿绱掓潏銊﹀鞍鐎垫澘瀚伴獮鍥敆閸屻倖袨濠电姷鏁搁崑娑樜涘▎鎾村仭鐟滃繒鍒?
// ==========================================================
//void display_right_angle_param(void)
//{
//    // 1. 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼崹顔惧帿闂佹悶鍊栧ú妯兼崲濠靛洨绡€闁稿本纰嶅▓顓犵磽娴ｆ彃浜鹃柣搴秵閸犳鎮″☉姘ｅ亾楠炲灝鍔氶柟鍐茬箻瀹曟澘螣濮瑰洣绨?//    lcd_showstr(0, 0, "RA Monitor");

//    // --- 闂傚倸鍊烽懗鍓佸垝椤栫偐鈧箓宕奸妷銉︽К闂佸搫绋侀崢濂告倿閸偁浜滈柟杈剧稻绾爼鏌涢弬璺ㄐｅǎ鍥э躬椤㈡稑顫濋崗澶广劎绱?---
//    lcd_showfloat(0, 2, right_angle_flag, 1, 0);
//    lcd_showstr(80, 2, "Flag"); // 缂傚倸鍊搁崐鎼佸磹閹间礁纾归柣銏㈩焾閻ょ偓绻涢幋娆忕仼闁? 闂傚倸鍊烽懗鍓佸垝椤栫偐鈧箓宕奸妷銉︽К闂佸搫绋侀崢濂告倿閸偁浜滈柟杈剧稻绾爼鏌涢弬璺ㄐч柡宀嬬秮瀵粙鎮介悽鏁屾氨绱?
//    lcd_showfloat(0, 3, gyro_roll_sign_angle, 1, 0);
//    lcd_showstr(80, 3, "G_SW"); // 缂傚倸鍊搁崐鎼佸磹閹间礁纾归柣銏㈩焾閻ょ偓绻涢幋娆忕仼闁? 闂傚倸鍊搁崐鎼佸磹閸濄儮鍋撳鈧崶褏锛涢梺鐟板⒔缁垶鎮￠敐澶嬬厸闁告洦鍋嗙粻鏌ユ煏閸℃鏆ｉ柡灞剧☉铻栭柍褜鍓氶〃銉╁箹娓氬洦鏅梺鎸庣箓椤︻垶鎮為崹顐犱簻闁圭儤鍨甸鈺冪磼閳?
//    // --- 闂傚倸鍊烽懗鍫曗€﹂崼銏″床闁瑰鍋熺粻鎯р攽閻樺弶鎼愰柡瀣╃劍閵囧嫰骞橀崡鐐典患缂佺偓鍎崇紞濠囧蓟閵堝悿鍦偓锝庡亝閻濇洜绱掗崜褍鐝￠柛銉ｅ妿閸?(闂傚倸鍊风粈渚€骞栭銈囩煋闁割偅娲栭崒銊ф喐韫囨拹? 闂備浇宕甸崰鎰垝鎼淬垺娅犳俊銈呮噹缁犱即鏌涘☉姗堟敾婵炲懐濞€閺岋絽螣濞嗘儳娈紒?/ 闂傚倸鍊风粈渚€骞栭銈囩煋闁哄鍤氬ú顏勎у璺猴躬濡嘲顪冮妶鍡欏⒈闁稿绋撴竟? ---

//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼崹顔惧帿闂佹娊鏀卞Λ鍐蓟瀹ュ牜妾ㄩ梺鍛婃尵閸犳牠鐛Δ鍛殝闁割煈鍋勯～锟犳⒑閸濆嫷妲搁柣蹇旂箘婢规洜鎹勯妸褏锛濇繛杈剧到婢瑰﹤螣閸儲鐓曢柕濞垮€曞畵鍡欌偓瑙勬礉椤骞嗛弮鍫熸櫜闁糕剝顨嗛悗顓㈡⒒娴ｅ憡璐￠柛搴涘€濆畷褰掓偨閸撳弶鏅?
//    lcd_showfloat(0, 4, gyro_right_angle, 3, 1);
//    lcd_showstr(45, 4, "/");
//    lcd_showfloat(55, 4, RIGHT_ANGLE_TARGET_ANGLE, 3, 0);
//    lcd_showstr(90, 4, "Ang"); // 缂傚倸鍊搁崐鎼佸磹閹间礁纾归柣銏㈩焾閻ょ偓绻涢幋娆忕仼闁? 闂傚倷娴囧畷鐢稿窗閹扮増鍋￠柨鏃傚亾閺嗘粓鏌熼悜妯荤厸闁?
//    // 闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊ノ熼崹顔惧帿闂佺粯甯掗…鐑藉蓟瀹ュ牜妾ㄩ梺鍛婃尵閸犲酣鎮惧畡鎵虫瀻闁规儳鍘栫槐鍫曟⒑閸涘﹥澶勯柛鎿勭畵瀹曟垿濡搁埡鍌楁嫼闂佸憡绻傜€氼剟寮抽悙鐑樼厱閹肩话鏉夸划濡炪們鍨洪敃銏ょ嵁閺嶃劍濯撮柛娑橈功閳?//    lcd_showfloat(0, 5, right_angle_count, 3, 0);
//    lcd_showstr(45, 5, "/");
//    lcd_showfloat(55, 5, RIGHT_ANGLE_EXIT_COUNT, 3, 0);
//    lcd_showstr(90, 5, "Cnt"); // 缂傚倸鍊搁崐鎼佸磹閹间礁纾归柣銏㈩焾閻ょ偓绻涢幋娆忕仼闁? 闂傚倷娴囧畷鍨叏瀹曞洦濯奸柡灞诲劚缁€鍫熺節闂堟侗鍎忔慨?//}

#define FLASH_SYSTEM_PARAMS_ADDR   0x000  // 闂傚倸鍊风粈渚€骞夐敓鐘冲殞濡わ絽鍟崑瀣煕閳╁啰鈯曢柛銈嗗姍閺岋綁寮崒姘粯闁荤喐鐟辩粻鎾荤嵁閺嶃劍缍囬柛鎾楀惙鎴︽⒑娴兼瑧绉ù婊冪埣瀵鍩勯崘鈺侇€撻梺鍛婄箓鐎氱兘寮抽銏″€甸悷娆忓鐏忣偆绱掗懜浣冨闁伙絿鍏橀幃婊堟寠婢跺矂鐛撻梻浣告贡椤牆霉妞嬪海鐭?
#define FLASH_MAGIC_ADDR           0x100  // 婵犳鍠楃敮鎺斺偓姘煎幖椤繗銇愰幒鎴炶緢闂佺粯姊婚崢褔骞戦崼鏇熺厸濠㈣泛顑呴婊呯磼?
#define FLASH_MAGIC_NUMBER         0x55AA55AAUL
// Global variables for menu state
int current_menu_level = 1; // 1 = Main Menu, 2 = Submenu
int main_menu_selection = 0; // Index of the highlighted item in the main menu (0 to 5 - 1)
int active_submenu = -1; // Which submenu is active (-1 if none)
uint8 key_value_test = 0; // Stores the result of key_scan

#define MAIN_MENU_ITEMS 6   // 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨堕弻锝夊箣閿濆憛鎾绘煛閸涱喗鍊愰柡灞剧洴楠炴鎲撮崟顓溾偓濠傤渻閵堝繒绉甸柛锝忕秮閻涱喛绠涘☉娆忊偓濠氭煠閹帒鍔滄繛鍛灲濮婃椽宕崟顐熷亾閸洖纾归柡宥庡亐閸嬫挸顫濋鍌溞ㄩ梺鍝勮閸旀垿骞冮姀銈呭窛濠电姴瀚槐鏇㈡⒒娴ｅ摜绉烘い銉︽崌瀹曟顫滈埀顒€顕ｉ锕€绠婚悹鍥у级椤ユ繈姊洪棃娑氬婵☆偅顨婇、?
#define CURSOR_X_POS    100   // 闂傚倷绀佸﹢閬嶁€﹂崼婢濇椽濡搁埡浣勶附鎱ㄥΟ鍨厫闁绘挶鍎甸弻锝夊棘閹稿骸鏆堥梺璇叉禋閸ｏ綁寮婚悢椋庢殝闁哄瀵т簺闂?"<<<" 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨伴—鍐偓锝庝簽閺勫倸霉閻樺磭娲存慨濠呮閹瑰嫰濡搁妷锔锯偓楣冩⒑閸濆嫷鍎忔い顓犲厴閻?X 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨堕弻锝夊箣閿濆憛鎾绘煛閸涱喗鍊愰柡宀嬬節瀹曟帒螣鐞涒€充壕闁哄稁鍋€閸?
#define CURSOR_STR      "< " // 闂傚倷绀佸﹢閬嶁€﹂崼婢濇椽濡搁埡浣勶附鎱ㄥΟ鍨厫闁绘挶鍎甸弻锝夊棘閹稿骸鏆堥梺璇叉禋閸ｏ綁寮婚悢椋庢殝闁哄瀵т簺闂備線娼уú锕€螞閸愵喖钃熼柕濞炬櫆閸嬪嫭绻涢懠顒傚笡闁哄拋鍙冨娲川婵犲倻浠ч梺绋匡攻閻楁粓鍩€椤掍浇澹樼紓宥咃躬瀵濡搁埡鍌氫簻闂佸憡绋戦敃锔藉閸曨垱鈷?(闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨堕弻锝夊箣閿濆憛鎾绘煛閸涱喗鍊愰柡宀嬬秮婵℃悂濡烽敂缁橈骏婵＄偑鍊х紞鍡涘闯閿濆宓侀煫鍥ㄧ⊕閸婂鏌ら幁鎺戝姕婵炲懌鍨藉娲传閸曨偀鍋撻崼鏇炵９闁哄稁鍋€閸嬫挸顫濋鍌溞ㄩ梺鍝勮閸旀垿骞冮姀銈呭窛濠电姴瀚槐鏇㈡⒒娴ｅ摜绉烘い銉︽崌瀹曟顫滈埀顒€顕ｉ锕€绠婚柟纰卞幗椤斿洭姊洪幆褏绠抽柟铏崌閹啴骞嬮敂鐣屽幍婵＄偛顑呮鎼佸储閹绢喗鐓涘ù锝堫潐瀹曞瞼鈧鍣崜鐔镐繆閸洖骞㈡俊銈咃梗缁憋箓鏌?
#define NO_CURSOR_STR   "  " // 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨堕弻锝夊箣閿濆憛鎾绘煛閸涱喗鍊愰柡宀嬬節瀹曟帒螣鐞涒€充壕闁哄稁鍋€閸嬫挸顫濋鍌溞ㄩ梺鍝勮閸旀垿骞冮姀銈呭窛濠电姴瀚槐鏇㈡⒒娴ｅ摜绉烘い銉︽崌瀹曟顫滈埀顒€顕ｉ锕€绠婚悹鍥у级椤ユ繈姊洪棃娑氬婵☆偅顨婇、鏃堝醇閺囩啿鎷洪柣鐘充航閸斿矂寮稿☉銏＄厱婵炲棙鍔栭妵婵堚偓瑙勬礃閻熲晜淇婇崼鏇炲耿婵°倕锕ｇ槐锕傛⒒娴ｅ搫鍔﹂柡鍛櫊瀹曚即寮介鐐电崶濠殿喗銇涢崑鎾绘煛鐏炶濮傞柟顔哄灲瀹曘劍绻濋崟顏嗙？闂傚倷鑳堕幊鎾诲床閺屻儱搴婇柟缁樺俯閻掍粙鏌涢幇闈涙灍闁绘挶鍎甸弻锝夊棘閹稿骸鏆堥梺璇叉禋閸ｏ綁寮婚悢鍓叉Ч閹肩补鈧啿绠ｉ梻?(缂傚倷鑳堕搹搴ㄥ矗鎼淬劌闂柨婵嗩槸閺嬩線鏌熼幑鎰靛殭婵☆偅锕㈤弻鏇㈠醇濠靛浂妫炲銈呯箰閻栫厧顫忛搹瑙勫磯闁靛鍎查悗楣冩⒑閸濆嫷鍎忔い顓犲厴閻涱喛绠涘☉娆忊偓濠氭煠閹帒鍔滄繛鍛灲濮婃椽宕崟顐熷亾閸洖纾归柡宥庡亐閸嬫挸顫濋鍌溞ㄩ悗瑙勬礈婵箖濡甸幇鏉跨闁规儳鐡ㄩ鐔兼⒒娴ｅ憡鎯堥柛濠傜埣瀹曟劙寮介鐔蜂壕?CURSOR_STR)

// --- 缂傚倷鑳堕搹搴ㄥ矗鎼淬劌闂柨婵嗩槸閺嬩線鏌熼幑鎰靛殭婵☆偅锕㈤弻鏇㈠醇濠靛浂妫炲銈呯箰閻栫厧顫忛搹瑙勫磯闁靛鍎查悗楣冩⒑閸濆嫷鍎忔い顓犲厴閻涱喛绠涘☉娆愬暎闂佺鏈划宥呂ｉ埀顒勬⒒娴ｄ警鐒剧紒璇茬墦瀹曟繈骞嬮敃鈧弸渚€鏌熼幑鎰厫闁稿蓱娣囧﹪濡堕崟顓фМ闂佽桨绶氱粻鏍蓟閻斿壊妲归幖绮光偓鍐茬闂備浇妗ㄩ悞锕傚箲閸パ屽殨妞ゆ帒瀚洿闂佺硶鍓濋〃蹇斿閹剧粯鈷掑ù锝堝Г绾爼鏌涢悩鍐插鐎殿喖鎲＄换婵嗩潩椤掑偆鍞甸梻浣虹帛閸ㄥ吋鎱ㄩ妶澶婄柧闁归棿鐒﹂悡娑㈡煕鐏炰箙顏堝焵椤掍胶澧遍柍褜鍓氶懝鍓х礊婵犲洤钃熼柕濞炬櫆閸嬪嫰鏌涘☉姗堝姛濞寸厧瀚板楦裤亹閹烘繃顥栭梺绋跨箲閿曘垹顕ｉ锕€绠婚悹鍥у级椤ユ繈姊洪棃娑氬婵☆偅顨婇、鏃堝醇閺囩啿鎷洪柣鐘充航閸斿矂寮搁幋锔界厸閻庯綆浜堕悡鍏碱殽閻愯尙绠婚柡灞诲妿閳ь剨缍嗘禍鍫曞焵椤掍胶澧垫慨濠呮閹瑰嫰濡搁妷锔锯偓楣冩⒑閸濆嫷鍎忔い顓犲厴閻涱喛绠涘☉娆忊偓濠氭煠閹帒鍔滄繛鍛灲濮婃椽宕崟顐熷亾閸洖纾归柡宥庡亐閸嬫挸顫濋鍌溞ㄩ悗瑙勬穿缁绘繈鐛弽銊﹀閻熸瑥瀚鐔兼⒒娴ｅ憡鎯堥柛濠傜埣瀹曟劙寮介鐔蜂壕婵﹩鍓涚粔娲煛鐏炶濮傞柟顔哄灲瀹曘劍绻濋崟顏嗙？闂傚倷绀佺紞濠囁夐幘璇茬婵せ鍋撶€?---
extern int main_menu_selection; // 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂佺媭鍨抽埀顒傛嚀鐎氼厼顭垮鈧矾闁告稑鐡ㄩ埛鎴炪亜閹扳晛鐒烘俊顖楀亾闂備浇妗ㄩ悞锕傚箲閸パ呮殾闁挎繂顦介弫鍐煏閸繃顥犻柡鍡欏█濮婃椽宕楅悡搴殝缂備緡鍠栭柊锝夊Υ閸岀儐鏁嬮柍褜鍓熼悰顔跨疀濞戞瑥鈧鏌ら幁鎺戝姕婵炲懌鍨藉娲传閸曨偀鍋撻崼鏇炵９闁哄稁鍋€閸嬫挸顫濋鍌溞ㄩ梺鍝勮閸旀垿骞冮姀銈呭窛濠电姴瀚槐鏇㈡⒒娴ｅ摜绉烘い銉︽崌瀹曟顫滈埀顒€顕ｉ锕€绠婚悹鍥у级椤ユ繈姊洪棃娑氬婵☆偅顨婇、?(0 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柟鐗堟緲閸戠姴鈹戦悩瀹犲缂?MAIN_MENU_ITEMS - 1)
extern const char *main_menu_options[MAIN_MENU_ITEMS]; // 闂傚倸鍊峰ù鍥р枖閺囥垹绐楅柡鍥╁枑閸欏繘鏌曢崼婵愭Ц缁炬儳銈搁弻鏇熺箾閻愵剚鐝曢梺鍝勬噺閹倿寮诲☉銏犵閻熸瑥瀚妴濠傤渻閵堝繒绉甸柛锝忕秮閻涱喛绠涘☉娆忊偓濠氭煠閹帒鍔滄繛鍛灲濮婃椽宕崟顐熷亾閸洖纾归柡宥庡亐閸嬫挸顫濋鍌溞ㄩ梺鍝勮閸旀垿骞冮姀銈呯畾闁哄顑欓崵瀣⒒娴ｇ瓔娼愮憸鏉垮暣椤㈡牠宕卞▎灞戒壕婵﹩鍓涚粔娲煛鐏炶濮傞柟顔哄灲瀹曘劍绻濋崟顏嗙？闂傚倷绀佺紞濠囁夐幘璇茬婵せ鍋撶€殿噮鍋婇獮妯兼嫚閸欏妫熼梻渚€娼ч悧鍡椢涘Δ鍜佹晜闁割偅娲橀埛鎴︽偣閹帒濡奸柡瀣灴閺岋紕鈧綆浜堕悡鍏碱殽?


//const char *main_menu_options[MAIN_MENU_ITEMS] = {
//    "R_PID",
//    "L_PID",
//    "TURN_PID",
//    "check",
//    "MOVE",
//		"ERROR"
//};


void save_all_params_to_flash(void)
{
    SystemParams buffer;
    unsigned long magic = FLASH_MAGIC_NUMBER;

    // Turn PID
    buffer.kp  = Turn_PID.kp;
    buffer.ki  = Turn_PID.ki;
    buffer.kd  = Turn_PID.kd;
    buffer.kp1 = Turn_PID.kp1;

    // Error params
    buffer.err_H  = err_H;
    buffer.err_X  = err_X;
    buffer.err_HM = err_HM;
    buffer.err_D  = err_D;
    buffer.err_M  = err_M;

    // Left motor PID
    buffer.L_kp = L_pid.kp;
    buffer.L_ki = L_pid.ki;
    buffer.L_kd = L_pid.kd;

    // Right motor PID
    buffer.R_kp = R_pid.kp;
    buffer.R_ki = R_pid.ki;
    buffer.R_kd = R_pid.kd;

    // Gyro PID
    buffer.G_kp  = Gyro_PID.kp;
    buffer.G_ki  = Gyro_PID.ki;
    buffer.G_kd  = Gyro_PID.kd;
    buffer.G_kp1 = Gyro_PID.kp1;

    // Circle thresholds闂傚倸鍊烽悞锔锯偓绗涘懐鐭欓柟杈鹃檮閸嬪鈹戦悩鍙夋悙闁藉啰鍠庨埞鎴︽偐閹绘帗娈插┑鐐叉噽婵炩偓闁哄矉缍侀獮瀣攽閸パ呯崺缂傚倷璁查崑?
    buffer.in_circle_LR = in_circle_LR;
    buffer.in_circle_MID = in_circle_MID;
    buffer.in_circle_LRMID = in_circle_LRMID;
		buffer.ring_error = ring_error;
		
		
		
    // 闂?闂傚倸鍊风粈渚€骞栭锕€纾归柣鐔煎亰閻斿棙鎱ㄥ璇蹭壕閻犱警鍨堕弻娑㈠箛闂堟稒鐏堥梺娲诲幗椤ㄥ﹪寮诲☉姘勃闁告挆鈧慨鍥р攽?
    buffer.ring_inc_element12 = ring_inc_element12;
    buffer.ring_inc_element56 = ring_inc_element56;
    buffer.ring_inc_element67 = ring_inc_element67;

    buffer.ring_angle_23 = ring_angle_23;
    buffer.ring_angle_34 = ring_angle_34;
    buffer.ring_angle_45 = ring_angle_45;

    buffer.temp_flag_tar = temp_flag_tar;;
		
		
				// 闂傚倷娴囬褏鈧稈鏅濈划娆撳箳濡や焦娅旈梻鍌欒兌椤牓顢栭崨顖滅煋鐟滅増甯掗拑鐔兼煕濞戞鎽犻柛瀣閳ь剛鎳撶€氫即宕戞繝鍥ц埞婵炲樊浜濋埛鎴﹀级閻愭潙顥嬮柛鏂跨Т椤潡鎮烽悧鍫￥闂?		buffer.speed[0] = speed[0];
		buffer.speed[1] = speed[1];
		buffer.speed[2] = speed[2];
		buffer.speed[3] = speed[3];
		buffer.speed[4] = speed[4];



    // 闂?闂傚倷娴囬褏鈧稈鏅濈划娆撳箳濡や焦娅旈梻鍌欒兌椤牓顢栭崨顖滅煋闁荤喐澹嗛弳锔戒繆閵堝懏鍣圭€瑰憡绻冮妵鍕冀閵娧€濮囧┑顕嗙到鐎氭澘顫忓ú顏呭€婚柍鍝勫€归悵鏍ь渻閵堝骸骞栭柣妤佹尭閻ｇ兘鏁愰崼顐ｂ枌闁诲氦顫夊ú鈺冪礊娴ｅ摜鏆﹂柟鐑樺灍濡插牊绻涢崱妯哄Е闁哥啿鍋?
    buffer.adc_vbat_tar = adc_vbat_tar;
    buffer.encoder_charge_element_vbat_tar = encoder_charge_element_vbat_tar;
    buffer.charge_pwm_open_val = charge_pwm_open_val;



    iap_erase_page(0); // 闂傚倸鍊风粈渚€骞夐敓鐘冲仭闁靛鏅涚壕鍦喐閻楀牆绗掓慨瑙勭叀閺岋綁寮埀顒勫箖閼哥數顩?    iap_erase_page(1); // 濠电姵顔栭崰妤冩暜閹烘柡鍋撳鐓庡箹妞ゎ偄绻楅妵鎰板箳閹寸偠绶㈤梻浣虹帛濮婂綊骞冮懜鐢殿洸?
    iap_write_bytes(FLASH_SYSTEM_PARAMS_ADDR, (unsigned char*)&buffer, sizeof(buffer));
    iap_write_bytes(FLASH_MAGIC_ADDR, (unsigned char*)&magic, sizeof(magic));
}





bit load_all_params_from_flash(void)
{
    SystemParams buffer;
    unsigned long magic;
    bit has_valid_data = 0;

    iap_read_bytes(FLASH_MAGIC_ADDR, (unsigned char*)&magic, sizeof(magic));

    if (magic == FLASH_MAGIC_NUMBER) {
        iap_read_bytes(FLASH_SYSTEM_PARAMS_ADDR, (unsigned char*)&buffer, sizeof(buffer));

        // Turn PID
        Turn_PID.kp  = buffer.kp;
        Turn_PID.ki  = buffer.ki;
        Turn_PID.kd  = buffer.kd;
        Turn_PID.kp1 = buffer.kp1;

        // Error weights
        err_H  = buffer.err_H;
        err_X  = buffer.err_X;
        err_HM = buffer.err_HM;
        err_D  = buffer.err_D;
        err_M  = buffer.err_M;

        // Left motor PID
        L_pid.kp = buffer.L_kp;
        L_pid.ki = buffer.L_ki;
        L_pid.kd = buffer.L_kd;

        // Right motor PID
        R_pid.kp = buffer.R_kp;
        R_pid.ki = buffer.R_ki;
        R_pid.kd = buffer.R_kd;

        // Gyro PID
        Gyro_PID.kp  = buffer.G_kp;
        Gyro_PID.ki  = buffer.G_ki;
        Gyro_PID.kd  = buffer.G_kd;
        Gyro_PID.kp1 = buffer.G_kp1;

        // 闂?闂傚倸鍊风粈渚€骞夐垾鎰佹綎缂備焦蓱閸欏繒鈧娲栧ú銊х不閺冨牊鐓曢悘鐐插⒔閳洖霉濠у灝鈧牜鎹㈠☉銏犲耿婵°倓绀侀弨顓㈡⒑閸愭彃妲婚柣妤佹崌楠炲啰鎹勭悰鈩冾潔濠碘槅鍨甸崑鎰板礉閸︻厾纾?        in_circle_LR  = buffer.in_circle_LR;
        in_circle_MID = buffer.in_circle_MID;
				in_circle_LRMID= buffer.in_circle_LRMID;
				ring_error= buffer.ring_error;
				
				
        ring_inc_element12 = buffer.ring_inc_element12;
        ring_inc_element56 = buffer.ring_inc_element56;
        ring_inc_element67 = buffer.ring_inc_element67;

        ring_angle_23 = buffer.ring_angle_23;
        ring_angle_34 = buffer.ring_angle_34;
        ring_angle_45 = buffer.ring_angle_45;


								// 闂傚倸鍊风粈渚€骞夐垾鎰佹綎缂備焦蓱閸欏繘鏌熼锝囦汗鐟滅増甯掗悙濠冦亜閹哄秷鍏岄柛鏃傛暬濮婅櫣绱掑Ο鑽ゅ弳闂佹悶鍨洪悡锟犮€佸▎鎰瘈闁搞儯鍔庨崢鍛婄箾鏉堝墽鎮奸柟铏崌钘熼悗锝庡墰绾?				speed[0] = buffer.speed[0];
				speed[1] = buffer.speed[1];
				speed[2] = buffer.speed[2];
				speed[3] = buffer.speed[3];
				speed[4] = buffer.speed[4];


        temp_flag_tar = buffer.temp_flag_tar;
				
				
				 // 闂?闂傚倸鍊风粈渚€骞夐垾鎰佹綎缂備焦蓱閸欏繘鏌熼锝囦汗鐟滅増甯掗悙濠冦亜閹哄棗浜剧紓浣插亾闁稿瞼鍋為悡鏇熺箾閹存繂鑸归柣蹇ョ節閺岋繝宕辫箛鏃€鐝氶梺鍝勭焿缂嶄線宕洪敓鐘茬闁告挆鍐ㄐ梻鍌欑閹诧繝顢旈幖浣哥倞鐟滃繘鎮￠幋锔解拺缂佸瀵у﹢鎵磼鐎ｎ偄绗掗柍?
        adc_vbat_tar = buffer.adc_vbat_tar;
        encoder_charge_element_vbat_tar = buffer.encoder_charge_element_vbat_tar;
        charge_pwm_open_val = buffer.charge_pwm_open_val;

				
        has_valid_data = 1;
    } else {
        
    }

    return has_valid_data;
}



/****************濠电姷鏁搁崑鐐哄垂閸洖绠插〒姘ｅ亾妞ゃ垺淇洪ˇ褰掓煙椤栨氨澧︾€规洖宕埥澶婎潩椤掑倶鍋?***************//****************濠电姷鏁搁崑鐐哄垂閸洖绠插〒姘ｅ亾妞ゃ垺淇洪ˇ褰掓煙椤栨氨澧︾€规洖宕埥澶婎潩椤掑倶鍋?***************//****************濠电姷鏁搁崑鐐哄垂閸洖绠插〒姘ｅ亾妞ゃ垺淇洪ˇ褰掓煙椤栨氨澧︾€规洖宕埥澶婎潩椤掑倶鍋?***************/


void display_submenu_check(uint8 key_press) {
    key_value_test = key_press;

    lcd_showstr(0, 0, "L:   ");
    lcd_showuint16(31, 0, L);
    lcd_showfloat(80, 0, adc_vbat, 3, 1);

    lcd_showstr(0, 1, "R:   ");
    lcd_showuint16(31, 1, R);

    lcd_showstr(0, 2, "LM:  ");
    lcd_showuint16(31, 2, LM);

    lcd_showstr(0, 3, "RM:  ");
    lcd_showuint16(31, 3, RM);

    lcd_showstr(0, 4, "MID: ");
    lcd_showuint16(31, 4, MID);

    lcd_showstr(0, 5, "Sum: ");
    lcd_showuint16(31, 5, L + R + LM + RM + MID);

    lcd_showstr(0, 6, "Err: ");
    lcd_showfloat(36, 6, Turn_PID.err, 5, 2);

    lcd_showstr(0, 7, "lv");
    lcd_showfloat(14, 7, l_speed_now, 4, 1);
    lcd_showstr(65, 7, "lT");
    lcd_showfloat(80, 7, L_pid.Target_base, 4, 1);

    lcd_showstr(0, 8, "rv");
    lcd_showfloat(14, 8, r_speed_now, 4, 1);
    lcd_showstr(65, 8, "rT");
    lcd_showfloat(80, 8, R_pid.Target_base, 4, 1);

    lcd_showstr(49, 9, "inc");
    lcd_showfloat(0, 9, mot_inc, 3, 1);

    adjust_parameter_by_key_float(key_value_test, &speed[0], (float)x_t_int * x_t_float);
    R_pid.Target_base = speed[0];
    L_pid.Target_base = R_pid.Target_base;
}


void display_circle_debug_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
        lcd_clear(WHITE);
    }

    if (key_mode == 0)
        adjust_parameter_by_key_float(key_value, &in_circle_LR, 1);
    else if (key_mode == 1)
        adjust_parameter_by_key_float(key_value, &in_circle_LRMID, 1);
    else if (key_mode == 2)
        adjust_parameter_by_key_float(key_value, &in_circle_MID, 1);
    else if (key_mode == 3)
        adjust_parameter_by_key_float(key_value, &ring_error, (float)x_t_int * x_t_float * 0.05f);

    lcd_showstr(0, 0, "Circle");

    lcd_showstr(0, 1, "LR:");
    lcd_showfloat(80, 1, in_circle_LR, 3, 1);
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "LRMID:");
    lcd_showfloat(80, 2, in_circle_LRMID, 3, 1);
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "MID:");
    lcd_showfloat(80, 3, in_circle_MID, 3, 1);
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "R_Err:");
    lcd_showfloat(80, 4, ring_error, 2, 3);
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "out:");
    lcd_showfloat(80, 5, temp_flag, 4, 0);

    lcd_showstr(0, 6, "Flag:");
    lcd_showuint8(80, 6, cir_flag);

    lcd_showstr(0, 7, "GSign:");
    lcd_showuint8(80, 7, gyro_roll_sign_rign);
}


void display_circle_advanced_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 7) key_mode = 0;
        lcd_clear(WHITE);
    }

    if (key_mode == 0)
        adjust_parameter_by_key_float(key_value, &ring_inc_element12, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 1)
        adjust_parameter_by_key_float(key_value, &ring_inc_element56, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 2)
        adjust_parameter_by_key_float(key_value, &ring_inc_element67, (float)x_t_int * x_t_float * 0.01f);
    else if (key_mode == 3)
        adjust_parameter_by_key_float(key_value, &ring_angle_23, (float)x_t_int * x_t_float);
    else if (key_mode == 4)
        adjust_parameter_by_key_float(key_value, &ring_angle_34, (float)x_t_int * x_t_float);
    else if (key_mode == 5)
        adjust_parameter_by_key_float(key_value, &ring_angle_45, (float)x_t_int * x_t_float);
    else if (key_mode == 6)
        adjust_parameter_by_key_float(key_value, &temp_flag_tar, (float)x_t_int * x_t_float);

    lcd_showstr(0, 0, "Circle Adv");

    lcd_showstr(0, 1, "inc12:");
    lcd_showfloat(65, 1, ring_inc_element12, 1, 2);
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "inc56:");
    lcd_showfloat(65, 2, ring_inc_element56, 1, 2);
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "inc67:");
    lcd_showfloat(65, 3, ring_inc_element67, 1, 2);
    lcd_showstr(120, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "ang23:");
    lcd_showfloat(65, 4, ring_angle_23, 3, 1);
    lcd_showstr(120, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "ang34:");
    lcd_showfloat(65, 5, ring_angle_34, 3, 1);
    lcd_showstr(120, 5, key_mode == 4 ? "<" : " ");

    lcd_showstr(0, 6, "ang45:");
    lcd_showfloat(65, 6, ring_angle_45, 3, 1);
    lcd_showstr(120, 6, key_mode == 5 ? "<" : " ");

    lcd_showstr(0, 7, "TempTar:");
    lcd_showfloat(65, 7, temp_flag_tar, 3, 0);
    lcd_showstr(120, 7, key_mode == 6 ? "<" : " ");
}


void display_speed_menu(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value = key_press;

    if (key_value == KEY_EVENT_ITEM_NEXT)
    {
        key_mode++;
        if (key_mode >= 5) key_mode = 0;
        lcd_clear(WHITE);
    }

    adjust_parameter_by_key_float(key_value, &speed[key_mode], (float)x_t_int * x_t_float);

    lcd_showstr(0, 0, "Speed Menu");

    lcd_showstr(0, 1, "Spd0:");
    lcd_showfloat(60, 1, speed[0], 3, 0);
    lcd_showstr(110, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "Spd1:");
    lcd_showfloat(60, 2, speed[1], 3, 0);
    lcd_showstr(110, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "Spd2:");
    lcd_showfloat(60, 3, speed[2], 3, 0);
    lcd_showstr(110, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "Spd3:");
    lcd_showfloat(60, 4, speed[3], 3, 0);
    lcd_showstr(110, 4, key_mode == 3 ? "<" : " ");

    lcd_showstr(0, 5, "Spd4:");
    lcd_showfloat(60, 5, speed[4], 3, 0);
    lcd_showstr(110, 5, key_mode == 4 ? "<" : " ");

    lcd_showstr(0, 7, "xT:");
    lcd_showfloat(40, 7, x_t_int * x_t_float, 3, 2);
}


void display_submenu_ee(uint8 key_press) {
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;

    if (key_value_test == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_value_test, &x_t_int, 2); break;
        case 1: adjust_parameter_by_key_float(key_value_test, &x_t_float, 0.1f); break;
        case 2: adjust_parameter_by_key_float(key_value_test, &err_t, (float)x_t_int * x_t_float * 0.00001f); break;
        default: break;
    }

    lcd_showfloat(0, 0, x_t_int, 2, 4);
    lcd_showstr(80, 0, " x_t ");
    lcd_showstr(120, 0, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 1, x_t_float, 2, 4);
    lcd_showstr(80, 1, " x_f ");
    lcd_showstr(120, 1, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 2, err_t, 2, 4);
    lcd_showstr(80, 2, " e_t ");
    lcd_showstr(120, 2, key_mode == 2 ? "<" : " ");

    lcd_showfloat(20, 9, x_t_int * x_t_float, 2, 3);
}


void display_gyro(uint8 key_press) {
    static unsigned char key_mode = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 7) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_press, &dir_loop_limit, (float)x_t_int * x_t_float); break;
        case 1: adjust_parameter_by_key_float(key_press, &dir_enlarge, (float)x_t_int * x_t_float); break;
        case 2: adjust_parameter_by_key_float(key_press, &err_H, (float)x_t_int * x_t_float * 0.1f); break;
        case 3: adjust_parameter_by_key_float(key_press, &err_X, (float)x_t_int * x_t_float * 0.1f); break;
        case 4: adjust_parameter_by_key_float(key_press, &err_HM, (float)x_t_int * x_t_float * 0.1f); break;
        case 5: adjust_parameter_by_key_float(key_press, &err_D, (float)x_t_int * x_t_float * 0.1f); break;
        case 6: adjust_parameter_by_key_float(key_press, &err_M, (float)x_t_int * x_t_float * 0.1f); break;
        default: break;
    }

    lcd_showfloat(0, 0, dir_loop_limit, 5, 2);
    lcd_showstr(70, 0, " mit");
    lcd_showstr(120, 0, key_mode == 0 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 1, dir_enlarge, 4, 2);
    lcd_showstr(70, 1, " lar");
    lcd_showstr(120, 1, key_mode == 1 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 2, err_H, 2, 4);
    lcd_showstr(70, 2, " errH");
    lcd_showstr(120, 2, key_mode == 2 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 3, err_X, 2, 4);
    lcd_showstr(70, 3, " errX");
    lcd_showstr(120, 3, key_mode == 3 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 4, err_HM, 2, 4);
    lcd_showstr(70, 4, " errC");
    lcd_showstr(120, 4, key_mode == 4 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 5, err_D, 2, 4);
    lcd_showstr(70, 5, " errD");
    lcd_showstr(120, 5, key_mode == 5 ? CURSOR_STR : NO_CURSOR_STR);

    lcd_showfloat(0, 6, err_M, 2, 4);
    lcd_showstr(70, 6, " errM");
    lcd_showstr(120, 6, key_mode == 6 ? CURSOR_STR : NO_CURSOR_STR);
}


void display_submenu_charge_debug(uint8 key_press) {
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;
    const char *state_str = "OFF ";
    const char *mode_str = "MAN ";
    const char *trigger_str = "NONE";

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_duty_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
    negative_pressure_auto_set_request_mode(0);
#endif

    if (key_value_test == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 7) key_mode = 0;
    }

    switch (key_mode) {
        case 0:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
                negative_pressure_auto_set_enabled(negative_pressure_auto_enabled ? 0 : 1);
            }
            break;
        case 1:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
                negative_pressure_auto_set_armed(negative_pressure_auto_armed ? 0 : 1);
            }
            break;
        case 2:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
                negative_pressure_auto_set_request_mode(0);
#else
                negative_pressure_auto_set_request_mode(negative_pressure_auto_use_circle_trigger ? 0 : 1);
#endif
            }
            break;
        case 3:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
                if (!negative_pressure_auto_use_circle_trigger) {
                    negative_pressure_auto_set_request(negative_pressure_auto_manual_request ? 0 : 1);
                }
            }
            break;
        case 4:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
                negative_pressure_auto_clear_lockout();
            }
            break;
        case 5:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC) {
                negative_pressure_auto_set_fault(negative_pressure_auto_fault_latched ? 0 : 1);
            }
            break;
        case 6:
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
            negative_pressure_duty_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#else
            if (key_value_test == KEY_EVENT_ADJ_INC) negative_pressure_duty_percent++;
            else if (key_value_test == KEY_EVENT_ADJ_DEC && negative_pressure_duty_percent > 0) negative_pressure_duty_percent--;
            if (negative_pressure_duty_percent < 5) negative_pressure_duty_percent = 5;
            if (negative_pressure_duty_percent > 10) negative_pressure_duty_percent = 10;
#endif
            break;
        default:
            break;
    }

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    mode_str = "MAN ";
#else
    mode_str = negative_pressure_auto_use_circle_trigger ? "CIRC" : "MAN ";
#endif

    switch (negative_pressure_auto_state) {
        case NEG_PRESS_AUTO_STATE_PREPARE: state_str = "PREP"; break;
        case NEG_PRESS_AUTO_STATE_HOLD:    state_str = "HOLD"; break;
        case NEG_PRESS_AUTO_STATE_RELEASE: state_str = "RELS"; break;
        case NEG_PRESS_AUTO_STATE_FAULT:   state_str = "FAULT"; break;
        case NEG_PRESS_AUTO_STATE_OFF:
        default:                           state_str = "OFF "; break;
    }

    switch (negative_pressure_auto_trigger_source) {
        case NEG_PRESS_TRIGGER_MENU_SIM: trigger_str = "MENU"; break;
        case NEG_PRESS_TRIGGER_CIRCLE:   trigger_str = "CIRC"; break;
        case NEG_PRESS_TRIGGER_VBAT:     trigger_str = "VBAT"; break;
        case NEG_PRESS_TRIGGER_ENCODER:  trigger_str = "ENCO"; break;
        case NEG_PRESS_TRIGGER_FAULT:    trigger_str = "FLT "; break;
        case NEG_PRESS_TRIGGER_NONE:
        default:                         trigger_str = "NONE"; break;
    }

    if(key_mode <= 6)
    {
        lcd_showstr(0, 0, "NP Auto A");

        lcd_showstr(0, 1, "EN:");
        lcd_showuint8(25, 1, negative_pressure_auto_enabled ? 1 : 0);
        lcd_showstr(45, 1, key_mode == 0 ? "<" : " ");
        lcd_showstr(65, 1, "AR:");
        lcd_showstr(95, 1, negative_pressure_auto_armed ? "A" : "S");
        lcd_showstr(110, 1, key_mode == 1 ? "<" : " ");

        lcd_showstr(0, 2, "MOD:");
        lcd_showstr(40, 2, mode_str);
        lcd_showstr(80, 2, key_mode == 2 ? "<" : " ");

        lcd_showstr(0, 3, "REQ:");
        lcd_showuint8(35, 3, negative_pressure_auto_request ? 1 : 0);
        lcd_showstr(50, 3, key_mode == 3 ? "<" : " ");
        lcd_showstr(70, 3, "CLR");
        lcd_showstr(105, 3, key_mode == 4 ? "<" : " ");

        lcd_showstr(0, 4, "LCK:");
        lcd_showuint8(35, 4, negative_pressure_auto_lockout ? 1 : 0);
        lcd_showstr(70, 4, "FLT:");
        lcd_showuint8(105, 4, negative_pressure_auto_fault_latched ? 1 : 0);
        lcd_showstr(120, 4, key_mode == 5 ? "<" : " ");

        lcd_showstr(0, 5, "SH:");
        lcd_showuint8(25, 5, negative_pressure_auto_shot_count);
        lcd_showstr(35, 5, "/");
        lcd_showuint8(45, 5, negative_pressure_auto_max_shots);
        lcd_showstr(70, 5, "CD:");
        lcd_showuint16(95, 5, negative_pressure_auto_cooldown_ticks_remaining);

        lcd_showstr(0, 6, "DY:");
        lcd_showuint8(25, 6, negative_pressure_duty_percent);
        lcd_showstr(50, 6, key_mode == 6 ? "<" : " ");
        lcd_showstr(70, 6, "TG:");
        lcd_showuint8(95, 6, negative_pressure_auto_target_duty_percent);

        lcd_showstr(0, 7, "ST:");
        lcd_showstr(25, 7, state_str);
        lcd_showstr(70, 7, "SRC:");
        lcd_showstr(105, 7, trigger_str);

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
        lcd_showstr(0, 8, "G:B1 P1 P33ON ");
#elif NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE
        lcd_showstr(0, 8, "G:B1 P0 P33OFF");
#elif NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
        lcd_showstr(0, 8, "G:B0 P1 P33OFF");
#else
        lcd_showstr(0, 8, "G:B0 P0 P33OFF");
#endif
        lcd_showstr(0, 9, "5% 100MS");
#else
        lcd_showstr(0, 8, "P71: OUT");
        lcd_showstr(0, 9, "NO FAN");
#endif
    }
}

void display_negative_pressure_output_debug(uint8 key_press)
{
    static unsigned char key_mode = 0;
    unsigned char key_value_test = key_press;
    const char *state_str = "OFF ";
    const char *stage2_map_str = "N";

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
    negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#endif

    if (key_value_test == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC)
            {
                negative_pressure_stage2_set_enable(negative_pressure_stage2_output_enable ? 0 : 1);
            }
            break;
        case 1:
#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
            negative_pressure_stage2_output_percent = NEGATIVE_PRESSURE_LOAD_TEST_FIXED_DUTY_PERCENT;
#else
            if (key_value_test == KEY_EVENT_ADJ_INC)
            {
                negative_pressure_stage2_output_percent++;
            }
            else if (key_value_test == KEY_EVENT_ADJ_DEC && negative_pressure_stage2_output_percent > 0)
            {
                negative_pressure_stage2_output_percent--;
            }
            if (negative_pressure_stage2_output_percent < 5) negative_pressure_stage2_output_percent = 5;
            if (negative_pressure_stage2_output_percent > 10) negative_pressure_stage2_output_percent = 10;
#endif
            break;
        case 2:
            if (key_value_test == KEY_EVENT_ADJ_INC || key_value_test == KEY_EVENT_ADJ_DEC)
            {
                if (negative_pressure_stage2_output_target == NEG_PRESS_STAGE2_MAP_NONE)
                {
                    negative_pressure_stage2_set_target(NEG_PRESS_STAGE2_MAP_P07);
                }
                else if (negative_pressure_stage2_output_target == NEG_PRESS_STAGE2_MAP_P07)
                {
                    negative_pressure_stage2_set_target(NEG_PRESS_STAGE2_MAP_FAN_Q2);
                }
                else
                {
                    negative_pressure_stage2_set_target(NEG_PRESS_STAGE2_MAP_NONE);
                }
            }
            break;
        default:
            break;
    }

    switch (negative_pressure_auto_state)
    {
        case NEG_PRESS_AUTO_STATE_PREPARE: state_str = "PREP"; break;
        case NEG_PRESS_AUTO_STATE_HOLD:    state_str = "HOLD"; break;
        case NEG_PRESS_AUTO_STATE_RELEASE: state_str = "RELS"; break;
        case NEG_PRESS_AUTO_STATE_FAULT:   state_str = "FAULT"; break;
        case NEG_PRESS_AUTO_STATE_OFF:
        default:                           state_str = "OFF "; break;
    }

    switch (negative_pressure_stage2_output_target)
    {
        case NEG_PRESS_STAGE2_MAP_P07:    stage2_map_str = "07"; break;
        case NEG_PRESS_STAGE2_MAP_FAN_Q2: stage2_map_str = "FAN"; break;
        case NEG_PRESS_STAGE2_MAP_NONE:
        default:                          stage2_map_str = "N"; break;
    }

    lcd_showstr(0, 0, "NP Out B");

    lcd_showstr(0, 1, "O2:");
    lcd_showstr(30, 1, negative_pressure_stage2_output_enable ? "1" : "0");
    lcd_showstr(50, 1, key_mode == 0 ? "<" : " ");

    lcd_showstr(0, 2, "P:");
    lcd_showuint8(25, 2, negative_pressure_stage2_output_percent);
    lcd_showstr(55, 2, key_mode == 1 ? "<" : " ");

    lcd_showstr(0, 3, "M:");
    lcd_showstr(25, 3, stage2_map_str);
    lcd_showstr(55, 3, key_mode == 2 ? "<" : " ");

    lcd_showstr(0, 4, "R:");
    lcd_showuint8(25, 4, negative_pressure_stage2_prepared_percent);

    lcd_showstr(0, 5, "RL:");
    lcd_showuint8(30, 5, negative_pressure_auto_real_output_percent);

    lcd_showstr(0, 6, "ST:");
    lcd_showstr(25, 6, state_str);

    lcd_showstr(0, 7, "LCK:");
    lcd_showuint8(35, 7, negative_pressure_auto_lockout ? 1 : 0);
    lcd_showstr(65, 7, "FLT:");
    lcd_showuint8(100, 7, negative_pressure_auto_fault_latched ? 1 : 0);

#if NEGATIVE_PRESSURE_LOAD_TEST_PROFILE_ENABLE
#if NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE && NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    lcd_showstr(0, 8, "G:B1 P1 P33ON ");
#elif NEGATIVE_PRESSURE_LOAD_TEST_BUILD_ENABLE
    lcd_showstr(0, 8, "G:B1 P0 P33OFF");
#elif NEGATIVE_PRESSURE_LOAD_TEST_PHYSICAL_OUTPUT_ENABLE
    lcd_showstr(0, 8, "G:B0 P1 P33OFF");
#else
    lcd_showstr(0, 8, "G:B0 P0 P33OFF");
#endif
    lcd_showstr(0, 9, "5% 100MS");
#else
    lcd_showstr(0, 8, "P33 EMPTY");
    lcd_showstr(0, 9, "NO FAN");
#endif
}

void display_straight_param(uint8 key_press)
{
    static unsigned char key_mode = 0;

    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 2) key_mode = 0;
    }

    switch (key_mode)
    {
        case 0:
            adjust_parameter_by_key_float(key_press, &straight_err_threshold, (float)x_t_int * x_t_float);
            break;
        case 1:
            adjust_parameter_by_key_float(key_press, &straight_integral_threshold, (float)x_t_int * x_t_float);
            break;
        default:
            break;
    }

    lcd_showstr(0, 0, "Straight");
    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

    lcd_showfloat(0, 1, straight_err_threshold, 2, 3);
    lcd_showstr(60, 1, " Err_Th ");
    lcd_showstr(120, 1, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 2, straight_integral_threshold, 2, 3);
    lcd_showstr(60, 2, " Int_Th ");
    lcd_showstr(120, 2, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 3, gyro_right_angle, 2, 3);
    lcd_showstr(60, 3, " rtang ");

    lcd_showfloat(0, 4, Turn_PID.err, 3, 3);
    lcd_showstr(89, 4, " Err ");

    lcd_showfloat(0, 5, encoder_straight_element, 3, 3);
    lcd_showstr(89, 5, " Int ");

    lcd_showstr(0, 7, "Flag:");
    lcd_showfloat(50, 7, right_angle_flag, 1, 0);
}

// Explicit monitor thresholds for the right-angle debug page.
#define RIGHT_ANGLE_TARGET_ANGLE 76.0f
#define RIGHT_ANGLE_EXIT_COUNT   40.0f

void display_right_angle_param(uint8 key_press)
{
    if (key_press == KEY_EVENT_ITEM_NEXT)
    {
        lcd_clear(WHITE);
    }

    lcd_showstr(0, 0, "RA Monitor");

    lcd_showfloat(0, 2, right_angle_flag, 1, 0);
    lcd_showstr(80, 2, "Flag");

    lcd_showfloat(0, 3, gyro_roll_sign_angle, 1, 0);
    lcd_showstr(80, 3, "G_SW");

    lcd_showfloat(0, 4, gyro_right_angle, 3, 1);
    lcd_showstr(45, 4, "/");
    lcd_showfloat(55, 4, RIGHT_ANGLE_TARGET_ANGLE, 3, 0);
    lcd_showstr(90, 4, "Ang");

    lcd_showfloat(0, 5, right_angle_count, 3, 0);
    lcd_showstr(45, 5, "/");
    lcd_showfloat(55, 5, RIGHT_ANGLE_EXIT_COUNT, 3, 0);
    lcd_showstr(90, 5, "Cnt");
}

float x_t_int=1;
float x_t_float=0.2;
/****************闂傚倸鍊风粈渚€宕ョ€ｎ兘鍋撶粭娑樻噹閸ㄦ繈鎮橀悙鍨珪闁哄棴绠撻弻锝夊籍閸屾艾浠樼紓浣插亾闁糕剝绋掗悡鏇㈡煏婢舵稓鍒板┑鈥炽偢楠?***************//****************闂傚倸鍊风粈渚€宕ョ€ｎ兘鍋撶粭娑樻噹閸ㄦ繈鎮橀悙鍨珪闁哄棴绠撻弻锝夊籍閸屾艾浠樼紓浣插亾闁糕剝绋掗悡鏇㈡煏婢舵稓鍒板┑鈥炽偢楠?***************//****************闂傚倸鍊风粈渚€宕ョ€ｎ兘鍋撶粭娑樻噹閸ㄦ繈鎮橀悙鍨珪闁哄棴绠撻弻锝夊籍閸屾艾浠樼紓浣插亾闁糕剝绋掗悡鏇㈡煏婢舵稓鍒板┑鈥炽偢楠?***************/

//void display(void)
//{
//    static unsigned char key_mode = 0;
//	  static unsigned char key_mode_in = -1;

//    unsigned char key_value_test;

//    key_value_test = key_scan(1);

//	        if (current_menu_level == 1) {
//            // --- Handle Main Menu ---
//            if (key_value_test == 3) { // P76 (Down Navigation)
//										main_menu_selection++;
//                if (main_menu_selection >= MAIN_MENU_ITEMS) {
//                    main_menu_selection = 0; // Wrap around
//                }
//                  key_scan(1); // IMPORTANT: Reset key state after processing navigation
//                  display_main_menu(); // Update display immediately
//            } else if (key_value_test == 4) { // P46 (Confirm/Enter)
//                current_menu_level = 2;
//                active_submenu = main_menu_selection;
//                key_scan(1);// IMPORTANT: Reset key state after processing action
//                lcd_clear(WHITE); // Clear screen for submenu
//            } 
//								display_main_menu();
//				} 
//	
//			
//				
//				
//			else if (current_menu_level == 2) {
//            // --- Handle Submenu ---
//            if (key_value_test == 1) { // P77 (Back)
//							
//                current_menu_level = 1;
//                active_submenu = -1;
//                key_value_test =key_scan(1); // IMPORTANT: Reset key state after processing action
//                lcd_clear(WHITE); // Clear screen for main menu
//                display_main_menu(); // Display main menu immediately
//            } else {
//							
//							
//                switch (active_submenu) {
//                    case 0: display_motor(&R_pid,r_speed_now,current_r_pwm_duty, key_value); break;
//                    case 1: display_motor(&L_pid,l_speed_now,current_l_pwm_duty, key_value); break;
//                    case 2: display_t(); break;
//                    case 3: display_submenu_check(); break;
//                    case 4: display_submenu_ee(); break;
//										case 5: display_gyro(key_value_test);break;
//									
//                    default:
//										lcd_showstr(0, 0, "Error: Invalid Submenu");
//										break;
//                }
//            }
//        }
//	
//	

//}

/**************************************************************************
 濠电姷鏁搁崑鐐哄垂閸洖绠伴柟缁㈠枛绾惧鏌熼幆褍顣崇痪鎹愭闇夐柨婵嗙墱濞兼劗绱撳鍡楃仼闁逞屽墲椤煤濠靛洨顩叉い鎺嶈兌椤╂煡鏌熼梻瀵割槮缂侇偄绉归弻宥堫檨闁告挻鐟╁﹢渚€姊洪崫鍕檨鐎光偓閳ь剟宕ｉ崱娑欌拺闂傚牊绋撴晶鏇熶繆椤愶綆娈滈柟顖氬暞閵堬綁宕橀埡鍐ㄥ箞?
**************************************************************************/
//KalmanFilter kf_L, kf_LM, kf_RM, kf_R, kf_MID;

////闂傚倸鍊风粈渚€骞夐敍鍕殰婵°倕鍟畷鏌ユ煕瀹€鈧崕鎴犵礊閺嶎厽鐓欓柣妤€鐗婄欢鑼磼閳?//void kalman_init(KalmanFilter* filter, float q, float r, float initial_value)
//{
//    filter->q = q;
//    filter->r = r;
//    filter->x = initial_value;
//    filter->p = 1.0f; // 闂傚倸鍊风粈渚€骞夐敍鍕殰婵°倕鍟畷鏌ユ煕瀹€鈧崕鎴犵礊閺嶎厽鐓欓柣妤€鐗婄欢鑼棯椤撴稑浜鹃梻鍌氼煬閸嬪嫬煤閿曗偓椤繗銇愰幒鎾充簵濠电偛妫欓崹鐔煎磻?//    filter->k = 0;
//}
//void kalman_filters_init(void)
//{
//    kalman_init(&kf_L,   0.01f, 3.0f, 0);
//    kalman_init(&kf_LM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_RM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_R,   0.01f, 3.0f, 0);
//    kalman_init(&kf_MID, 0.01f, 3.0f, 0);
//}

////闂傚倸鍊风粈渚€骞栭鈷氭椽濡舵径瀣槐闂侀潧艌閺呮盯鎷戦悢灏佹斀闁绘ê寮堕幖鎰磼閳ь剟宕熼娑氬幘闂佽鍘界敮鎺楀礉濠婂嫮绠?
//float kalman_update(KalmanFilter* filter, float measurement)
//{
//    // 濠电姷顣藉Σ鍛村磻閸涱収鐔嗘俊顖氱毞閸嬫挸顫濋悡搴＄缂備緡鍠栭敃顏堢嵁濮椻偓椤㈡瑩鎮℃惔锛勩偖闂備浇顕х€涒晝绮欓幒鏇熸噷闂備胶绮幐鎾磻?
//    filter->p = filter->p + filter->q;

//    // 闂傚倷娴囧畷鍨叏瀹曞洦顐介柕鍫濇处椤洟鏌￠崶銉ョ仾闁稿鏅涢埞鎴︽偐鐎圭姴顥濈紒缁㈠幐閸嬫捇姊绘担鍝勪缓闁稿氦浜竟鏇㈩敇閻愭劖甯掗…銊╁醇閻斿搫骞愰梻浣规偠閸庮噣寮插☉姘殰闁煎摜鍋ｆ禍婊堟煏閸繃鍣归柡鍡秮閺?//    filter->k = filter->p / (filter->p + filter->r);

//    // 闂傚倸鍊风粈渚€骞栭鈷氭椽濡舵径瀣槐闂侀潧艌閺呮盯鎷戦悢灏佹斀闁绘ɑ褰冮弳娆愩亜閺囶澀鎲鹃柡宀€鍠庨悾锟犲箥椤旀儳濮奸梻浣告啞閸╁啴宕戦幘缁樷拻?//    filter->x = filter->x + filter->k * (measurement - filter->x);

//    // 闂傚倸鍊风粈渚€骞栭鈷氭椽濡舵径瀣槐闂侀潧艌閺呮盯鎷戦悢灏佹斀闁绘ɑ褰冮顏堟倵濮橆厾顣查柕鍥у瀵粙濡歌閺嗭繝姊哄畷鍥╁笡婵炶尙鍠栧璇测槈濡粍妫冮崺鈧い鎺戝€瑰畷鏌ユ煙鏉堝墽鐣遍柦鍐枛閺岋綁寮▎鐐枃闂?//    filter->p = (1 - filter->k) * filter->p;

//    return filter->x;
//}


//		// 闂傚倸鍊风粈渚€骞夐敓鐘参︽繛鎴欏灩閻掑灚銇勯幒鍡椾壕缂佸墽铏庨崢鑹邦暰濠电姴锕ら悧濠囨偂濞戙垺鐓曢柕澶嬪灥閸燁垶宕愰鐐粹拺缁绢厼鎳愰崼顏堟煕婵犲啯绀嬮柛鈹垮灲瀵噣宕煎┑鎰秱闂備胶绮悷銏ゅ磻閹剧繝绻嗛柡灞诲劙閹查箖鏌?//		L   = kalman_update(&kf_L,   L);
//		LM  = kalman_update(&kf_LM,  LM);
//		RM  = kalman_update(&kf_RM,  RM);
//		R   = kalman_update(&kf_R,   R);
//		MID = kalman_update(&kf_MID, MID);














