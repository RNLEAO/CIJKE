#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "isr.h"

uint8 key_mode_local =0;
/****************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨姗嗗厴閺€锕傛煃?***************//****************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨姗嗗厴閺€锕傛煃?***************//****************闂傚倷鐒﹂惇褰掑垂瑜版帗鍊舵慨姗嗗厴閺€锕傛煃?***************/
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

//    // 闂傚倷绀侀幉锛勬暜閹烘嚚娲晝閳ь剟鎮鹃悜鑺ュ殤妞ゆ帒鍊婚敍婵嬫⒑閸濆嫮鈻夐柛鎾寸〒閺侇噣宕卞Δ濠冨瘜闂侀潧鐗嗗ú锕傤敂閻旇櫣纾奸弶鍫涘妿閹冲懘鏌熸搴♀枅妞ゃ垺绋戦～婵嬵敇濞戞瑧浜為梻鍌氬€搁崐鎼佸疮椤愶附鍋嬮柛鈩冪☉閻?/ 缂傚倸鍊风粈渚€藝娴犲鍨傚ù鍏兼綑閸ㄥ倿骞栧ǎ顒€濡介柍閿嬫崌閺屾盯顢曢敐鍥ь洭闂佺顑嗛幐鎯р槈閸偒妲归幖杈剧秵濡?
//    if (key_value_test == 3) {
//        lcd_clear(WHITE);
//        key_mode++;
//        if (key_mode >= 2) key_mode = 0; // 闂傚倷绀侀幉锟犳偡椤栨稓顩叉繝闈涙４閼板灝霉閿濆懏璐￠柍缁樻閺屾洟宕煎┑鍡╀純闂佹悶鍊曢澶愬蓟濞戞ǚ鏋庨煫鍥ㄦ尨閸嬫捁銇愰幒鎴炶緢?//    }

//    // 闂傚倷绀侀幉锟犳偡閵夆晛纾圭憸鐗堝笒濮规煡鏌ｉ弮鍌氬妺閻庢凹鍓熼弻娑㈠箛闂堟稑绠归柧?
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

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鎸庣【闁告劏鍋撻梻浣规偠閸庢椽宕滈敃鍌氭辈?
//    lcd_showstr(0, 0, "straight");
//    lcd_showfloat(70, 0, x_t_int * x_t_float, 2, 3);

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鍧楀摵閻庢碍纰嶉妵鍕冀閵娧屾殹闂佸磭绮ú鐔奉潖婵犳艾绀冮柕濞у喚鏆梻?//    lcd_showfloat(0, 1, straight_err_threshold, 2, 3);
//    lcd_showstr(60, 1, " Err_Th ");
//    if (key_mode == 0) lcd_showstr(120, 1, "<"); else lcd_showstr(120, 1, " ");

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ澶稿惈闁活厽鎸婚妵鍕冀閵娿劌顥濋梺璇查椤嘲顫忔繝姘闁靛ě鍐炬毇闂?//    lcd_showfloat(0, 2, straight_integral_threshold, 2, 3);
//    lcd_showstr(60, 2, " Int_Th ");
//    if (key_mode == 1) lcd_showstr(120, 2, "<"); else lcd_showstr(120, 2, " ");
//		
//	 lcd_showfloat(0, 3, gyro_right_angle, 2, 3);
//   lcd_showstr(60, 3, " rtang ");

//		

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鐓庝缓濞存粌缍婇弻鐔煎箚瑜嶉弳杈ㄣ亜閵堝懏鍤囬柟顔肩秺瀹曞墎鎹勯搹閫涘摋婵犵妲呴崹浼存晝椤忓嫮鏆﹂柟閭﹀枟閸忔粌顪冪€ｎ亝鎹ｆい锔哄絾urn_PID.err闂?//    lcd_showfloat(0, 4, Turn_PID.err, 3, 3);
//    lcd_showstr(89, 4, " Err ");

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鐓庝缓濞存粌缍婇弻鐔煎箚瑜嶉弳杈ㄣ亜閵堝懏鍣界紒杈ㄥ笚瀵板嫬螣閾忛€涘摋闂備焦鎮堕崝宀勬煀閿濆懐鏆?//    lcd_showfloat(0, 5, encoder_straight_element, 3, 3);
//    lcd_showstr(89, 5, " Int ");

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鍨厫闁稿鍔欓弻銈夊传閵夘喗姣岄梺绋款儐閹稿骞忛崨鏉戞嵍妞ゆ挾鍋熸导鍥⒒婵犲骸浜滄繛璇х畵婵＄敻鎮欓悽鐢殿槸?//    lcd_showstr(0, 7, "Flag:");
//    if (right_angle_flag) lcd_showstr(50, 7, "1"); else lcd_showstr(50, 7, "0");
//}


// 闂備浇顕у锕傦綖婢舵劖鍎楁い鏂垮⒔娑撳秹鏌ｉ弬鍨倯闁稿骸娴烽惀顏堝箚瑜滈崕娑欑箾绾绡€闁哄本鐩獮鎺斺偓锝庝簴閸嬫捇寮介鐐舵憰闂佹悶鍎崝搴ㄥ煝閺冣偓閵囧嫰骞樼€靛摜鐣甸梺瀹狀嚙缁绘﹢寮婚敐澶涚稏妞ゆ巻鍋撳┑鈩冨缁绘盯宕奸悢椋庝紝闂佹悶鍔嶉崕鎶解€﹂妸鈺佺妞ゆ挾鍋熸导鍥⒒娴ｅ憡鍟為柛銊╂涧铻炴俊銈呮噹閻掑灚銇勯幒鍡椾壕闂佹眹鍔庨崗姗€宕洪埀?#define RIGHT_ANGLE_TARGET_ANGLE    76.0f   // 闂傚倷绀侀幉锛勬暜閸ヮ剙纾归柡宥庡幖閽冪喖鏌涢妷锝呭濠殿喗绮嶉妵鍕箳閸℃ぞ澹曢梻鍌氬€稿Λ妤€螞濞戙垹鐒垫い鎺戝€归崵鈧梺鍝勭墱閸撶喎鐣烽悷閭︽僵闁兼悂娼ч崜顓㈡⒑閸涘﹥澶勯柛妯兼櫕缁絽螖閸涱喚鍘搁悗骞垮劚閹冲繒绮氶幐搴涗簻妞ゆ劧缍嗗▓鏇㈡煙?#define RIGHT_ANGLE_EXIT_COUNT      40      // 闂備礁鎼ˇ閬嶅磿閹版澘绀堟繛鍡樻尰閳锋牠鏌涢埄鍐喛闁稿鎹囬幃鐑芥偋閸喐瀚抽梻浣规偠閸斿秴鐣濈粙璺ㄦ殾闁靛闄勯崕鐔搞亜閺嶃劎鐭屾い锔诲櫍閹鎮欓悷鎵冲亾濞戙垹鏄ラ柡宓本瀵岄梺绋跨灱閸嬬偤鎮炴繝姘厓鐟滄粓宕滈悢椋庢殾婵炲棙鎸婚幆鐐烘偡濞嗗繐顏柡鍡欏█閺岋綁鎮╂潏顐敼闂佸憡鎸荤换鍫濐嚕閹惰姤鍋勯柣鎾虫捣閿?// ==========================================================
// 闂傚倷绀侀幉锟犲垂閸忓吋鍙忛柕鍫濐槸濮规煡鏌ｉ弮鍌氬付閻熸瑱濡囬埀顒€绠嶉崕杈╂崲閹烘梻鐜绘慨濠冩敯splay_right_angle_param
// 闂傚倷绀侀幉鈥愁潖婵犳艾绐楅柡鍥ュ灩缁€鍌涙叏濡炶浜鹃梺? 闂傚倷绶氬濠氭嚈瑜版帒鐤柟璇插婵犵數鍋為崹鍫曞箰閹间焦鏅濋柕澶堝剻閻旂绶炵€光偓閳ь剟鎯岄崱娑欑叄闊洦鍑瑰鎰磼缂併垹鐏﹂柟顔煎槻閳诲酣骞囬鍌ゅ剮婵犵數鍋炲娆徫涘Δ鍜佹晪闁挎繂顦介弫宥嗙節婵犲倸顏柡鍡欏█濮婅櫣绮欑捄銊ь唹婵犵數鍋涢敃顏勵嚕椤愶富鏁囬柕蹇曞Х閻撴垵鈹戦濮愪粶闁稿鎹囬弻娑㈡晲閸℃ǜ浠㈡繝纰樷偓宕囨憼闁瑰嘲鎳樺畷顐﹀礋閹搭垳鐣甸柡宀€鍠栭、鏍箰鎼搭喗鈻婂┑鐐茬摠缁秶鏁垾宕囨殾闁挎繂妫涚弧鈧梺绋胯閸婃洖鈻撻幖浣圭厵闁绘挸娴烽幗鐘虫叏婵犲拋鍤欓柍缁樻崌楠炲鏁冮埀顒傜矆閸℃稒鐓忛柛顐ｇ箖椤ョ偤鏌ｉ幒宥囩煓闁哄备鈧剚鍚嬮柛娑卞枟閹瑩姊洪棃鈺冪У闁搞劌鐖煎顐㈩吋婢跺﹪鍞跺┑鐘诧工閸熺娀寮歌箛娑欌拺婵炶尙绮繛鍥煕閺傛寧鎹ｇ紒杈ㄦ尰瀵板嫰骞囬鍌ゆО婵犵數鍋涘Λ娆撴偡瑜忕划?
// ==========================================================
//void display_right_angle_param(void)
//{
//    // 1. 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鍨厫闁搞倖娲樼换婵囩節閸屾碍娈紓浣插亾閻庯綆鍠楅悡娑氣偓骞垮劚閹冲繘宕板Ο姹囦簻?//    lcd_showstr(0, 0, "RA Monitor");

//    // --- 闂傚倷鑳剁划顖炩€﹂崼銉ユ槬闁哄稁鍘奸悞鍨亜閹达絾纭堕柛鏂跨У娣囧﹪顢涘鍏夹ㄧ紓?---
//    lcd_showfloat(0, 2, right_angle_flag, 1, 0);
//    lcd_showstr(80, 2, "Flag"); // 缂傚倸鍊搁崐鎼佸磹閻㈢鐤炬繛鎴欏灩閻? 闂傚倷鑳剁划顖炩€﹂崼銉ユ槬闁哄稁鍘奸悞鍨亜閹达絾纭堕柛鏂跨Ч閺岋綁寮介悽鐢敌氱紓?
//    lcd_showfloat(0, 3, gyro_roll_sign_angle, 1, 0);
//    lcd_showstr(80, 3, "G_SW"); // 缂傚倸鍊搁崐鎼佸磹閻㈢鐤炬繛鎴欏灩閻? 闂傚倸鍊搁崐鍝モ偓姘€鍥х；闁瑰墽绮悡锝夋煛閸曨偆绠查柕鍡楀暣閺屾稑螖閳ь剚顨ラ幖渚囨晪闁挎繂顦悞鍨亜閹烘垵顏╃紒鈧?
//    // --- 闂傚倷鑳堕…鍫㈡崲閹寸偟绠惧┑鐘叉搐閺嬩焦銇勯幘鍗炵仼缁炬儳缍婇弻銈吤圭€ｎ偅鐝曠紒鍓у珡閸ャ劎鍘?(闂傚倷绀侀幖顐ょ矓閸洖鍌ㄧ憸蹇撐? 闂佽崵鍠愮划搴㈡櫠濡ゅ懎绠伴柛娑橈攻濞呯娀鏌ｅΟ娆惧殭缁?/ 闂傚倷绀侀幖顐ょ矓閺夋嚚娲Χ婢跺﹪妫峰銈嗙墱閸嬫稓澹? ---

//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鍨厫闁抽攱妫冮弻宥堫檨闁告挾鍠栭獮妤呮嚋閸偄顫￠梺鍝勵槸閻忔繄澹曠捄銊х＝濞达絽澹婂Ο鍫ユ煕閵娿倕宓嗙€规洝顫夐幆鏃堟晲閸℃鐎梻浣告贡閸庛倝宕归悽鍓叉晩?
//    lcd_showfloat(0, 4, gyro_right_angle, 3, 1);
//    lcd_showstr(45, 4, "/");
//    lcd_showfloat(55, 4, RIGHT_ANGLE_TARGET_ANGLE, 3, 0);
//    lcd_showstr(90, 4, "Ang"); // 缂傚倸鍊搁崐鎼佸磹閻㈢鐤炬繛鎴欏灩閻? 闂備浇宕甸崰鎰版偡閿旂偓鏆滈柟鐑樻煛閸?
//    // 闂傚倷绀侀幖顐も偓姘煎墯閺呰埖绂掔€ｎ€附鎱ㄥΟ鍨厫闁绘帒顭烽弻宥堫檨闁告挾鍠庨悾宄扳枎閹惧厖绱堕梺鍛婃处閸擄箓宕戦妸鈺傗拺闁告繂瀚弳鐐烘煕鎼绰板仮妞ゃ垺锕㈤獮鏍ㄦ媴閸涘﹦鈧?//    lcd_showfloat(0, 5, right_angle_count, 3, 0);
//    lcd_showstr(45, 5, "/");
//    lcd_showfloat(55, 5, RIGHT_ANGLE_EXIT_COUNT, 3, 0);
//    lcd_showstr(90, 5, "Cnt"); // 缂傚倸鍊搁崐鎼佸磹閻㈢鐤炬繛鎴欏灩閻? 闂備浇宕垫慨宕囨媼閺屻儱绀堟繝闈涱儏濮?//}

#define FLASH_SYSTEM_PARAMS_ADDR   0x000  // 闂傚倷绀侀幉锟犳嚌妤ｅ啯鍋嬮柛鈩冪⊕閸ゆ劙鏌ｉ弮鍌氬付閻熸瑱绠撻獮鏍ㄦ綇閸撗吷戦梺浼欑秮娴滃爼寮婚埄鍐╁闁告繂瀚烽弳顓㈡倵鐟欏嫭灏紒鑸佃壘閻ｇ兘鎮滈挊澶岄獓闂佸湱顭堝ù椋庣矈?
#define FLASH_MAGIC_ADDR           0x100  // 濠殿喗甯掔€氼厼顫忚ぐ鎺戞辈闁绘梻鍘ч幑鍫曟煛婢跺顕滅紒?
#define FLASH_MAGIC_NUMBER         0x55AA55AAUL
// Global variables for menu state
int current_menu_level = 1; // 1 = Main Menu, 2 = Submenu
int main_menu_selection = 0; // Index of the highlighted item in the main menu (0 to 5 - 1)
int active_submenu = -1; // Which submenu is active (-1 if none)
uint8 key_value_test = 0; // Stores the result of key_scan

#define MAIN_MENU_ITEMS 6   // 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屾盯骞樼憴鍕€婂銈忕秵閸ｏ綁鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢?
#define CURSOR_X_POS    100   // 闂備礁婀遍…鍫澝洪妸鈺佄︽慨妯垮煐閻撱儵鏌ｉ弬鎸庡暈闁诲浚鍣ｉ弻鐔风暋閺夋寧些闂?"<<<" 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垰顪冪€ｎ亞鏄傚ù鐘崇洴濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛?X 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋?
#define CURSOR_STR      "< " // 闂備礁婀遍…鍫澝洪妸鈺佄︽慨妯垮煐閻撱儵鏌ｉ弬鎸庡暈闁诲浚鍣ｉ弻鐔风暋閺夋寧些闂侀潧娲﹀Λ鍐蓟閵娾晜鍋勬繛鑼帛閺咁參姊洪崨濠傜仧闁稿﹥鐗滈埀顒佽壘缂嶅﹪寮婚妸鈺傚亜闁告稑锕︽导鍕⒑?(闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋綁濡搁妷锔绘￥濡炪倧缍嗛崳锝夌嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻閹碱厽顔囬梺鎯х箳閹虫捇鎯冮幋锔界厵濡炲楠搁崢鎾煛娴ｈ宕岀€殿噮鍓熸俊鍫曞幢濡ゅ﹣绱﹂柣?
#define NO_CURSOR_STR   "  " // 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸娑㈡煕濞嗘劖銇濈€规洘鐟╂俊鍫曞幢濡ゅ﹣绱﹂梻浣哄劦閺呪晠宕伴弽顓炵獥婵せ鍋撻柡灞诲姂閹垽宕ㄦ繝鍕磿闂備胶鎳撻崲鏌ュ床閹绘帩鐒介柛鎰靛枟閻撱儵鏌ｉ弬鎸庡暈闁诲浚鍣ｉ弻鐔割槹鎼粹€冲箣闂?(缂備胶铏庨崣搴ㄥ闯閿濆鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш鐎规洜濮鹃妵鎰板箳閹惧瓨顏熼梻浣告惈閸婂爼宕愰弽顐熷亾?CURSOR_STR)

// --- 缂備胶铏庨崣搴ㄥ闯閿濆鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欐啣闁稿本绮嶅Σ鈧梻浣侯焾缁诲牓宕濋幋锕€鏋侀柟鎹愬煐閸嬫ɑ淇婇妶鍕槮闁轰線绠栭弻鐔割槹鎼粹€冲箣闂佽桨鐒﹂幑鍥ь嚕椤掑嫬围闁糕剝顨忔导鎾绘⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佺粯鍨兼慨銈夊疾閹间焦鐓涢柛灞久埀顒佺墱閳ь剚鑹剧紞濠囧蓟閵娾晜鍋勯柛娑橈功娴煎嫰姊鸿ぐ鎺濇闁稿繑锕㈠顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閺屻劎鈧綆浜堕埀顒佺墵濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш鐎规洩绻濋獮鏍ㄦ媴鐟欏嫭顏熼梻浣告惈閸婂爼宕愰弽顐熷亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵?---
extern int main_menu_selection; // 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳鈧懓瀚姗€路閸涘瓨鈷戞い鎰╁焺濡偓闂佽桨鐒﹂幑鍥х暦閿濆鏁冮柕鍫濇閺嗙娀姊洪崗鐓庮嚋缂侇喖閰ｉ妴鍌烆敋閳ь剟鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢?(0 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁?MAIN_MENU_ITEMS - 1)
extern const char *main_menu_options[MAIN_MENU_ITEMS]; // 闂傚倷娴囧▔鏇㈠窗閺囩喐鍙忛柕鍫濐槸绾惧ジ鏌曟繛鐐珕闁哄懏鎮傞弻娑㈠箻鐟欏嫮銆婂銈忕秵閸ｏ綁鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ箚閺夋鍤嬮梻浣筋潐瑜板啴顢栭崱娆屽亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閸洘鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼?


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

    // Circle thresholds闂傚倷鐒︾€笛呯矙閹达附鍋嬪┑鐘叉搐閽冪喎鈹戦悩鎻掓殲婵炲懐濞€閺岋綁骞嬪┑鍥х獩缂備讲鍋?
    buffer.in_circle_LR = in_circle_LR;
    buffer.in_circle_MID = in_circle_MID;
    buffer.in_circle_LRMID = in_circle_LRMID;
		buffer.ring_error = ring_error;
		
		
		
    // 闂?闂傚倷绀侀幖顐﹀磹閻熼偊鐔嗘慨妞诲亾鐠侯垶鏌涢幇闈涙灈闁活厽顨婇弻娑氫沪閸撗€濮囧┑?
    buffer.ring_inc_element12 = ring_inc_element12;
    buffer.ring_inc_element56 = ring_inc_element56;
    buffer.ring_inc_element67 = ring_inc_element67;

    buffer.ring_angle_23 = ring_angle_23;
    buffer.ring_angle_34 = ring_angle_34;
    buffer.ring_angle_45 = ring_angle_45;

    buffer.temp_flag_tar = temp_flag_tar;;
		
		
				// 闂備浇顕х€涒晝绮欓幒妤佹櫔闂備胶顭堥鍛矓瑜版帒钃熼柛娑樼摠閸嬪嫮鈧懓瀚伴崑濠囧船濞差亝鈷戦弶鐐村閸斿秴顭块悷鐗堫棤闁?		buffer.speed[0] = speed[0];
		buffer.speed[1] = speed[1];
		buffer.speed[2] = speed[2];
		buffer.speed[3] = speed[3];
		buffer.speed[4] = speed[4];



    // 闂?闂備浇顕х€涒晝绮欓幒妤佹櫔闂備胶顭堥鍛矓閻熸壆鏆︽俊銈呮噹瀹告繃銇勯弽銊р姇婵絽瀚板娲倻閳哄倹鐝栧銈庡幖閻楁挸鐣烽敐鍫▌閻庤娲╃紞浣哥暦閹烘垟妲堟繛鍡樺姦閸熲偓
    buffer.adc_vbat_tar = adc_vbat_tar;
    buffer.encoder_charge_element_vbat_tar = encoder_charge_element_vbat_tar;
    buffer.charge_pwm_open_val = charge_pwm_open_val;



    iap_erase_page(0); // 闂傚倷绀侀幉锟犳偡閵夆晛纾圭憸鐗堝笒濮规煡鏌ｉ弮鈧幃鑸电?    iap_erase_page(1); // 婵犳鍠楃敮鎺斺偓姘煎幖椤繗銇愰幒鎴炶緢闂佺粯姊归幃鑸电?
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

        // 闂?闂傚倷绀侀幉鈥愁潖缂佹ɑ鍙忕€规洖娲ㄧ粻鏃堟煕鐏炲墽鈯曞ù婧垮€栫换娑㈠幢濡や礁鏀梺鍐插槻閻楁捇骞冪捄琛℃婵☆垵鍋愰崝鍦磽?        in_circle_LR  = buffer.in_circle_LR;
        in_circle_MID = buffer.in_circle_MID;
				in_circle_LRMID= buffer.in_circle_LRMID;
				ring_error= buffer.ring_error;
				
				
        ring_inc_element12 = buffer.ring_inc_element12;
        ring_inc_element56 = buffer.ring_inc_element56;
        ring_inc_element67 = buffer.ring_inc_element67;

        ring_angle_23 = buffer.ring_angle_23;
        ring_angle_34 = buffer.ring_angle_34;
        ring_angle_45 = buffer.ring_angle_45;


								// 闂傚倷绀侀幉鈥愁潖缂佹ɑ鍙忛柟顖ｇ亹瑜版帒鐐婃い鎺嶈兌閸旂敻姊虹紒妯荤叆闁搞垺鐓￠、娆愮節閸ャ劎鍘告繛杈剧悼閹虫捇藟鐎ｎ剛纾?				speed[0] = buffer.speed[0];
				speed[1] = buffer.speed[1];
				speed[2] = buffer.speed[2];
				speed[3] = buffer.speed[3];
				speed[4] = buffer.speed[4];


        temp_flag_tar = buffer.temp_flag_tar;
				
				
				 // 闂?闂傚倷绀侀幉鈥愁潖缂佹ɑ鍙忛柟顖ｇ亹瑜版帒鐐婃い鎺嗗亾缂佲偓閸岀偞鐓曟繛鎴濆船閻忥繝鏌￠崱蹇旀珚闁哄矉缍侀崺锟犲礃閸撗冨Ъ闂備礁鎲￠鎼佸炊瑜忛悡鎴︽⒑缁嬫寧婀扮紒瀣笒閳?
        adc_vbat_tar = buffer.adc_vbat_tar;
        encoder_charge_element_vbat_tar = buffer.encoder_charge_element_vbat_tar;
        charge_pwm_open_val = buffer.charge_pwm_open_val;

				
        has_valid_data = 1;
    } else {
        
    }

    return has_valid_data;
}



/****************婵犵數鍋為崹鍫曞箲娓氣偓椤㈡俺顦归柟顖氱墦瀹曞崬鈽夊顒傘偊?***************//****************婵犵數鍋為崹鍫曞箲娓氣偓椤㈡俺顦归柟顖氱墦瀹曞崬鈽夊顒傘偊?***************//****************婵犵數鍋為崹鍫曞箲娓氣偓椤㈡俺顦归柟顖氱墦瀹曞崬鈽夊顒傘偊?***************/


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

    if (key_value_test == KEY_EVENT_ITEM_NEXT) {
        lcd_clear(WHITE);
        key_mode++;
        if (key_mode >= 3) key_mode = 0;
    }

    switch (key_mode) {
        case 0: adjust_parameter_by_key_float(key_value_test, &adc_vbat_tar, 0.2f); break;
        case 1: adjust_parameter_by_key_float(key_value_test, &encoder_charge_element_vbat_tar, (float)x_t_int * x_t_float * 0.2f); break;
        case 2: adjust_parameter_by_key_float(key_value_test, &charge_pwm_open_val, (float)x_t_int); break;
        default: break;
    }

    lcd_showfloat(0, 0, adc_vbat_tar, 2, 2);
    lcd_showstr(60, 0, " Vtar ");
    lcd_showstr(120, 0, key_mode == 0 ? "<" : " ");

    lcd_showfloat(0, 1, encoder_charge_element_vbat_tar, 2, 2);
    lcd_showstr(60, 1, " ENC_tar ");
    lcd_showstr(120, 1, key_mode == 1 ? "<" : " ");

    lcd_showfloat(0, 2, charge_pwm_open_val, 4, 0);
    lcd_showstr(60, 2, " PWM ");
    lcd_showstr(120, 2, key_mode == 2 ? "<" : " ");

    lcd_showfloat(0, 4, encoder_charge_sign, 2, 1);
    lcd_showstr(52, 4, " ENC_SIGN ");

    lcd_showfloat(0, 5, pwm_state_charge, 2, 1);
    lcd_showstr(52, 5, " PWM_STATE ");

    lcd_showfloat(0, 6, encoder_charge_element, 2, 4);
    lcd_showstr(52, 6, " ENC_ELE ");
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

void display_right_angle_param(uint8 key_press)
{
    (void)key_press;

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
/****************闂傚倷绀侀崥瀣偓绗涘懎鍨濋悘鐐垫櫕閺嗭箓鏌ｉ弮鍌氬付缂佲偓閸℃稒鐓曢柕澶涚到婵″ジ骞?***************//****************闂傚倷绀侀崥瀣偓绗涘懎鍨濋悘鐐垫櫕閺嗭箓鏌ｉ弮鍌氬付缂佲偓閸℃稒鐓曢柕澶涚到婵″ジ骞?***************//****************闂傚倷绀侀崥瀣偓绗涘懎鍨濋悘鐐垫櫕閺嗭箓鏌ｉ弮鍌氬付缂佲偓閸℃稒鐓曢柕澶涚到婵″ジ骞?***************/

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
 婵犵數鍋為崹鍫曞箰閹绢喖纾婚柟鎯у绾捐棄霉閿濆牜娼愮紓宥嗗灩閳ь剝顫夊ú婵囩椤掍胶顩查柟闂寸缁秹鏌嶈閸撴瑩婀侀梺鍝勮癁瀹€鈧崣鍡涙⒑闂堟稓澧曟俊顐ｎ殜閹啯銈ｉ崘鈺冨幈?
**************************************************************************/
//KalmanFilter kf_L, kf_LM, kf_RM, kf_R, kf_MID;

////闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柛宀€鍎戠紞鏍煙閻楀牊绶茬紒鈧?//void kalman_init(KalmanFilter* filter, float q, float r, float initial_value)
//{
//    filter->q = q;
//    filter->r = r;
//    filter->x = initial_value;
//    filter->p = 1.0f; // 闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柛宀€鍎戠紞鏍煙閻楀牊绶茬痪顓涘亾闂傚鍋勫ú锕€顫忚ぐ鎺撳亗婵炲棙鍨熼崑?//    filter->k = 0;
//}
//void kalman_filters_init(void)
//{
//    kalman_init(&kf_L,   0.01f, 3.0f, 0);
//    kalman_init(&kf_LM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_RM,  0.01f, 3.0f, 0);
//    kalman_init(&kf_R,   0.01f, 3.0f, 0);
//    kalman_init(&kf_MID, 0.01f, 3.0f, 0);
//}

////闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶鎼愮紒鈧崟顖涚厾闁诡厽甯掗崝婊勭箾?
//float kalman_update(KalmanFilter* filter, float measurement)
//{
//    // 婵犵妲呴崑鍛熆濡皷鍋撳鐓庡箻缂侇喖锕獮姗€顢欓悡搴＄ギ闂佽瀛╃粙鎺曟懌闂佺粯鎸撮崑?
//    filter->p = filter->p + filter->q;

//    // 闂備浇宕垫慨宕囨閵堝洦顫曢柡鍥ュ灪閸嬧晛鈹戦悩瀹犲缁绢厸鍋撻梻浣哄仺閸庤京澹曢鐐愭帒顭ㄩ崼鐔哄幐闂佹悶鍎弲娑氭暜閼哥偣浜滈柕鍫濇噹閺嗭綁鏌?//    filter->k = filter->p / (filter->p + filter->r);

//    // 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樻彃鏆欐い鏇憾閺岀喎鐣￠幍顔惧姼闂佸憡鍩冮崑鎾绘⒒?//    filter->x = filter->x + filter->k * (measurement - filter->x);

//    // 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樻彃顏悗姘閵囧嫰寮介妸褜鏆￠梺宕囩帛濞茬喖寮诲☉妯滄棃鍩€椤掑倹宕查柟杈剧畱閽冪喖鏌ｉ弬娆炬疇闁?//    filter->p = (1 - filter->k) * filter->p;

//    return filter->x;
//}


//		// 闂傚倷绀侀幉锟犲Φ濞戙垹鐒垫い鎺嗗亾缁剧虎鍘艰婵犲﹤鐗婇悡娑㈡煕閵夋垵鍟崐顖炴⒑绾懐鍫柛濠冩礋閸┿垽寮崼婵愭綂闂佺粯鐟㈤崑鎾翠繆閺屻儰鎲鹃柡?//		L   = kalman_update(&kf_L,   L);
//		LM  = kalman_update(&kf_LM,  LM);
//		RM  = kalman_update(&kf_RM,  RM);
//		R   = kalman_update(&kf_R,   R);
//		MID = kalman_update(&kf_MID, MID);













