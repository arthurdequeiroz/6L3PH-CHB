#include <windows.h>
#include <cmath>
#include "dll.h"


//constantes
double h=0.000001, w=376.991118431, p23=2.09439510239, hpwm = 0.0001, pi=3.14159265;
//Entradas
double vlm=0.0, E=0.0, teta=0.0, vgm=0.0, eg1=0.0, delta=0.0, tf=0.0, egm=0.0, ig3=0.0, ig2=0.0, ig1=0.0, eg2=0.0, eg3=0.0, vc=0.0;
//vari?veis da estrat?gia de PWM
double ts = 0.0, derv = 0.0, vserra=0.0, sinal=0.0, migt=0.5, vg1_ref=0.0, vg2_ref=0.0, vg3_ref=0.0, vl1_ref=0.0, vl2_ref=0.0, vl3_ref=0.0, vgmax=0.0, 
vgmin=0.0, vgt_max=0.0, vgt_min=0.0, vgt=0.0, vr1=0.0, vr2=0.0, vr3=0.0, vl1=0.0, vl2=0.0, vl3=0.0, ang=0.0, vr1_ref=0.0, vr2_ref=0.0, vr3_ref=0.0,
vl10_ref=0.0, vl20_ref=0.0, vl30_ref=0.0; 
//vari?veis do controle de barramento
double kic = 8.0;//10
double kpc = 0.25;//0.25
double xic=0.0, xica=0.0, erB=0.0, erBa=0.0, vcr=0.0; 
//vari?veis do controle de corrente
double kii = 1000;//1000
double kpi = 1;//1
double ig1_r=0.0, ig2_r=0.0, ig3_r=0.0, igm=0.0, er1=0.0, er2=0.0, er1a=0.0, er2a=0.0, xaa1=0.0,xa1=0.0,xb1=0.0,xaa2=0.0,xa2=0.0,xb2=0.0;
double F1 = cos(hpwm*w), F2 = sin(hpwm*w)/w, F3 = 2.0*kii*sin(hpwm*w)/w, 
F4=-w*sin(w*hpwm), F5=cos(w*hpwm), F6=(cos(hpwm*w)-1.0)*2.0*kii;
//estados das chaves
double qg1=0.0, qg2=0.0, qg3=0.0, ql1=0.0, ql2=0.0, ql3=0.0;
//frequ?ncia m?dia
double sg=0.0, sl=0.0, qgant=0.0, qlant=0.0, fmg=0.0, fml=0.0;
 ///////////////// PLL eg1//////////////////////////////
double A1t = 0.0, A2t = 0.0, A3t = 1.0, A4t = 0.0;
double B1t = 1., B2t = 0.0, C1t = 0.55, C2t = 1.0, D1t = 0.0;
double A1 = -8.796*5., A2 = -39.48*5.*5., A3 = 1.0, A4 = 0.0;
double B1 = 1., B2= 0.0, C1 = 0.0, C2 = 39.48*5.*5., D1 = 0.0;
double Pd = 0.0, theta = 0.0, theta_i = 0.0, theta_f = 0.0;
double wff = 376.9911, X1t = 0.0, X2t = 0.0, dX1t = 0.0, dX2t = 0.0;
double X1tf = 0.0, X2tf = 0.0, Pdf = 0.0, e_pd = 0.0;
double X1a = 0., X2a = 0.0, dX1a = 0.0, dX2a = 0.0;
double X1af = 0.0, X2af = 0.0, kix = 0.0, kpx = 0.0, wf = 0.0, ww=10.; 
 ///////////////// PLL vg1//////////////////////////////
double A1tg = 0.0, A2tg = 0.0, A3tg = 1.0, A4tg = 0.0;
double B1tg = 1., B2tg = 0.0, C1tg = 0.55, C2tg = 1.0, D1tg = 0.0;
double Pdg = 0.0, thetag = 0.0;
double X1tg = 0.0, X2tg = 0.0, dX1tg = 0.0, dX2tg = 0.0;
double X1tfg = 0.0, X2tfg = 0.0, Pdfg = 0.0, e_pdg = 0.0;
double X1ag = 0., X2ag = 0.0, dX1ag = 0.0, dX2ag = 0.0;
double X1afg = 0.0, X2afg = 0.0, wfg=0.0; 
//transit?rio de carga
double flag_switch = 0.0;
//gambiarras
double angle_g;
//ITH
double vs0m1, vs0M1, vs0m2, vs0M2, vs0m3, vs0M3, vs0m, vs0M, mign;

//Controle CHB
// PWM
// Variáveis com sufixos de fase
double vll10_ref_1 = 0., vlr10_ref_1 = 0.; // FB1 - Fase 1
double vll20_ref_1 = 0., vlr20_ref_1 = 0.; // FB2 - Fase 1

double vll10_ref_2 = 0., vlr10_ref_2 = 0.; // FB1 - Fase 2
double vll20_ref_2 = 0., vlr20_ref_2 = 0.; // FB2 - Fase 2

double vll10_ref_3 = 0., vlr10_ref_3 = 0.; // FB1 - Fase 3
double vll20_ref_3 = 0., vlr20_ref_3 = 0.; // FB2 - Fase 3


// FBs Bus Control - FBs Voltage - Simple PI
// Fase 1
double vC2L1s_1 = 0., vC2L2s_1 = 0.;
double vC2L1s_ref_1 = 100., vC2L2s_ref_1 = 70.;
double vC2L_ref_1 = (vC2L1s_ref_1 + vC2L2s_ref_1) / 2;
double vC2L1s_error_1 = 0., vC2L2s_error_1 = 0.;
double v2L1_ref_1 = 0., v2L2_ref_1 = 0., V2L1_ref_1 = 0., V2L2_ref_1 = 0.;
double Iv2L1_error_1 = 0., Iv2L2_error_1 = 0., Pv2L1_error_1 = 0., Pv2L2_error_1 = 0.;
double P2L1_1 = 0., P2L2_1 = 0., Pltot_1 = 0.;
double iC2L1s_1 = 0., iC2L2s_1 = 0.;

// Fase 2
double vC2L1s_2 = 0., vC2L2s_2 = 0.;
double vC2L1s_ref_2 = 100., vC2L2s_ref_2 = 70.;
double vC2L_ref_2 = (vC2L1s_ref_2 + vC2L2s_ref_2) / 2;
double vC2L1s_error_2 = 0., vC2L2s_error_2 = 0.;
double v2L1_ref_2 = 0., v2L2_ref_2 = 0., V2L1_ref_2 = 0., V2L2_ref_2 = 0.;
double Iv2L1_error_2 = 0., Iv2L2_error_2 = 0., Pv2L1_error_2 = 0., Pv2L2_error_2 = 0.;
double P2L1_2 = 0., P2L2_2 = 0., Pltot_2 = 0.;
double iC2L1s_2 = 0., iC2L2s_2 = 0.;

// Fase 3
double vC2L1s_3 = 0., vC2L2s_3 = 0.;
double vC2L1s_ref_3 = 100., vC2L2s_ref_3 = 70.;
double vC2L_ref_3 = (vC2L1s_ref_3 + vC2L2s_ref_3) / 2;
double vC2L1s_error_3 = 0., vC2L2s_error_3 = 0.;
double v2L1_ref_3 = 0., v2L2_ref_3 = 0., V2L1_ref_3 = 0., V2L2_ref_3 = 0.;
double Iv2L1_error_3 = 0., Iv2L2_error_3 = 0., Pv2L1_error_3 = 0., Pv2L2_error_3 = 0.;
double P2L1_3 = 0., P2L2_3 = 0., Pltot_3 = 0.;
double iC2L1s_3 = 0., iC2L2s_3 = 0.;

// Controladores PI
double kpvl_1 = 0.3, kivl_1 = 8; // Ganhos iguais para todos os PIs 0.1 e 10
double kpvl_2 = 0.7, kivl_2 = 20;
double kpvl_3 = 0.7, kivl_3 = 20;

// FBs Bus Control - Load Current - Simple PI
// Fase 1
double ils_1 = 0., ils_ref_1 = 0.;
double Il_ref_1 = 15., Il_ref0_1 = 5.; // IL* = Eg*/S* = 9.1 A = 1 p.u; S* = 1000 W  --- 13.6 -> 1.5 p.u.
double v2Ls_max_1 = 0., v2Ls_ref_max_1 = 0., v2Ls_error_max_1 = 0.;
double kpil_1 = 1.0, kiil_1 = 10.; // Ganhos do controlador PI
double Iil_error_1 = 0., Pil_error_1 = 0.; // Erros integral e proporcional

// Fase 2
double ils_2 = 0., ils_ref_2 = 0.;
double Il_ref_2 = 15., Il_ref0_2 = 5.; // IL* = Eg*/S* = 9.1 A = 1 p.u; S* = 1000 W  --- 13.6 -> 1.5 p.u.
double v2Ls_max_2 = 0., v2Ls_ref_max_2 = 0., v2Ls_error_max_2 = 0.;
double kpil_2 = 1.0, kiil_2 = 10.; // Ganhos do controlador PI
double Iil_error_2 = 0., Pil_error_2 = 0.; // Erros integral e proporcional

// Fase 3
double ils_3 = 0., ils_ref_3 = 0.;
double Il_ref_3 = 15., Il_ref0_3 = 5.; // IL* = Eg*/S* = 9.1 A = 1 p.u; S* = 1000 W  --- 13.6 -> 1.5 p.u.
double v2Ls_max_3 = 0., v2Ls_ref_max_3 = 0., v2Ls_error_max_3 = 0.;
double kpil_3 = 1.0, kiil_3 = 10.; // Ganhos do controlador PI
double Iil_error_3 = 0., Pil_error_3 = 0.; // Erros integral e proporcional


int sign_1 = 0, sign_2 = 0, sign_3 = 0;

// Load Current Control - Predictive
double vlL_1, vlL_2, vlL_3 = 0.;
double Rl = 0.1, Ll = 5e-3; // Inductance Parameters

bool qg2L1_1, ql2L1_1, qg2L2_1, ql2L2_1, qg2L1_2, ql2L1_2, qg2L2_2, ql2L2_2, qg2L1_3, ql2L1_3, qg2L2_3, ql2L2_3;

double iccc = 10;
double igm2;

	
#define DLLIMPORT __declspec (dllexport)
extern "C" DLLIMPORT void OpenSimUser(const char *szId, const char * szNetlist, int nInputCount, int nOutputCount, int *pnError, char * szErrorMsg){
}

extern "C" DLLIMPORT void RunSimUser(double t, double delt, double *in, double *out, int *pnError, char * szErrorMsg){
	
	 tf = in[0];
     E = in[1];
     teta = in[2];
     egm = in[3];
     eg1 = in[4];
     vc = in[5];
     ig3 = in[6];
     ig2 = in[7];
     ig1 = in[8];
     eg2 = in[9];
     eg3 = in[10];
	 hpwm = in[11];
	 
	 vC2L1s_1 = in[12];
	 vC2L2s_1 = in[13];
	 vC2L1s_2 = in[14];
	 vC2L2s_2 = in[15];
	 vC2L1s_3 = in[16];
	 vC2L2s_3 = in[17];
	 
	 iC2L1s_1 = in[18];
	 iC2L2s_1 = in[19];
	 iC2L1s_2 = in[20];
	 iC2L2s_2 = in[21];
	 iC2L1s_3 = in[22];
	 iC2L2s_3 = in[23];
	 
	 ils_1 = in[24];
	 ils_2 = in[25];
	 ils_3 = in[26];	 
    
    derv = 2/hpwm;
    
    F1 = cos(hpwm*w); F2 = sin(hpwm*w)/w; F3 = 2.0*kii*sin(hpwm*w)/w; 
	F4=-w*sin(w*hpwm); F5=cos(w*hpwm); F6=(cos(hpwm*w)-1.0)*2.0*kii;
	
	bool key = 1;
    
	//loop discreto
	if(ts <= t){
		ts = ts + hpwm;
		vserra = -0.5;
		sinal = 1.0;
		
	//Transitório de carga
	if(t>0.0) flag_switch = 1.0;
		
    // =============== PLL de eg1 ============================================  
    kix = 2.5;//4.5;//3.5;//2.5;//1.0; //2.5
    kpx = 0.25;//0.55;//0.25//0.55

	Pd = 0.5*eg1*cos(theta);//(vn1+vgnm)
    //FPB
	X1a = X1af;
	X2a = X2af;
	
	dX1a = A1*X1a + A2*X2a + B1*Pd;
	dX2a = A3*X1a + A4*X2a + B2*Pd;
	
	X1af = X1a + dX1a*hpwm;
	X2af = X2a + dX2a*hpwm;
	
	Pdf = C1* X1af + C2*X2af + D1*Pd; //Saída do Filtro

	//Erro
	e_pd = -(0. - Pdf);

	//Controlador PI + Integrador
				  
	X1t = X1tf;
	X1tf = X1t + kix*e_pd*hpwm;
	dX1t = X1tf + kpx*e_pd;
	wf = wff + dX1t;
	
	theta = theta + wff*hpwm; //Saida da PLL
	   
	if(theta>=2.*pi) theta = theta - 2.*pi;
	if(theta<= 0.) theta = theta + 2.*pi; 
	
	//Controle de barramento
		vcr = E;
		xica = xic;
		erBa = erB;
		erB = vcr - vc;
		xic = xica + kic*hpwm*erBa;
		igm = xic + kpc*erB;
		
		if(igm > 200) igm = 200;
		if(igm < -200) igm = -200;
	
	// ===========================================================
//		Controle de corrente
//		igm2 = 1.0*30.0*sqrt(2);
//		igm = igm2;
		
		ig1_r = igm*sin(theta);
		ig2_r = igm*sin(theta-p23);
		ig3_r = igm*sin(theta+p23);
		
		er1a = er1;
		er2a = er2;
		er2 = -ig2_r + ig2;
		er1 = -ig1_r + ig1;
		
		xaa1 = xa1;
		xa1 = F1*xaa1+F2*xb1+F3*er1a;
		xb1 = F4*xaa1+F5*xb1+F6*er1a;
		
		if(xa1 > 2*E) xa1 = 2*E;
		if(xa1 < -2*E) xa1 = -2*E;
		vg1_ref = xa1 + 2*kpi*er1;
		
	    xaa2 = xa2;
		xa2 = F1*xaa2+F2*xb2+F3*er2a;
		xb2 = F4*xaa2+F5*xb2+F6*er2a;
		if(xa2 > 2*E) xa2 = 2*E;
		if(xa2 < -2*E) xa2 = -2*E;
		vg2_ref = xa2 + 2*kpi*er2;
									
		vg3_ref = -vg1_ref-vg2_ref;
		
		if(vg1_ref > 1*E) vg1_ref = 1*E;
		if(vg1_ref < -1*E) vg1_ref = -1*E;
		if(vg2_ref > 1*E) vg2_ref = 1*E;
		if(vg2_ref < -1*E) vg2_ref = -1*E;
		if(vg3_ref > 1*E) vg3_ref = 1*E;
		if(vg3_ref < -1*E) vg3_ref = -1*E;
		
//		vg1_ref = 150*sqrt(2)*sin(theta+angle_g);  //tensão de referência na carga fase 1
//        vg2_ref = 150*sqrt(2)*sin(theta - p23+angle_g);  //tensão de referência na carga fase 2
//        vg3_ref = 150*sqrt(2)*sin(theta + p23+angle_g);  //tensão de referência na carga fase 3
//        
//###################################### FASE 1 ######################################		
		
		// Cálculo de potência - Fase 1
		if(key){
			P2L1_1 = vC2L1s_ref_1 * iC2L1s_1;
			P2L2_1 = vC2L2s_ref_1 * iC2L2s_1;
		}
		else{
			P2L1_1 = vC2L1s_ref_1 * iccc;
			P2L2_1 = vC2L2s_ref_1 * iccc;
			
		}
		

		Pltot_1 = P2L1_1 + P2L2_1;
		
		if (Pltot_1 > 0) 
			sign_1 = 1;
		else 
			sign_1 = -1;

		// Controle de barramento - Fase 1
		//2L1
		vC2L1s_error_1 = sign_1 * (vC2L1s_ref_1 - vC2L1s_1);
		Iv2L1_error_1 = Iv2L1_error_1 + kivl_1 * hpwm * vC2L1s_error_1;
		Pv2L1_error_1 = kpvl_1 * vC2L1s_error_1;
		V2L1_ref_1 = (Iv2L1_error_1 + Pv2L1_error_1);
		
		v2L1_ref_1 = V2L1_ref_1 * sin(theta);
//		v2L1_ref_1 = 10 * sin(theta);
		
		//2L2
		vC2L2s_error_1 = sign_1 * (vC2L2s_ref_1 - vC2L2s_1);
		Iv2L2_error_1 = Iv2L2_error_1 + kivl_1 * hpwm * vC2L2s_error_1;
		Pv2L2_error_1 = kpvl_1 * vC2L2s_error_1;
		V2L2_ref_1 = (Iv2L2_error_1 + Pv2L2_error_1);
		
		v2L2_ref_1 = V2L2_ref_1 * sin(theta);
//		v2L2_ref_1 = 8 * sin(theta);
		
		
		// 2L1 PWM
		vll10_ref_1 =  (v2L1_ref_1 / 2) / vC2L1s_ref_1;
		vlr10_ref_1 = -(v2L1_ref_1 / 2) / vC2L1s_ref_1;
		
		// 2L2 PWM 	
		vll20_ref_1 =  (v2L2_ref_1 / 2) / vC2L2s_ref_1;
		vlr20_ref_1 = -(v2L2_ref_1 / 2) / vC2L2s_ref_1;
		
		//Controle de corrente il			
		if (abs(V2L1_ref_1) > abs(V2L2_ref_1))
		{
			v2Ls_max_1 = V2L1_ref_1;
			v2Ls_ref_max_1 = 0.9 * vC2L1s_ref_1; // Modulation Index ma = 0.9
		}
		else 
		{
			v2Ls_max_1 = V2L2_ref_1;
			v2Ls_ref_max_1 = 0.9 * vC2L2s_ref_1; // Modulation Index ma = 0.9
		}
		
		// PI
		v2Ls_error_max_1 = -(abs(v2Ls_ref_max_1) - abs(v2Ls_max_1));
		Iil_error_1 = Iil_error_1 + kiil_1 * hpwm * v2Ls_error_max_1;
		Pil_error_1 = kpil_1 * v2Ls_error_max_1;
		Il_ref_1 = sign_1 * (Iil_error_1 + Pil_error_1 + Il_ref0_1);
//			Il_ref_1 = (Iil_error_1 + Pil_error_1 + Il_ref0_1);
		
		// By the Power
//			if(abs(iC2L1s_1) > abs(iC2L2s_1)) //P2L1_1/vC2L1s_ref_1 = iC2L1s_1
//				Il_ref_1 = sqrt(2)*abs(iC2L1s_1);
//			else				//iC2L1s_1 < iC2L2s_1
//				Il_ref_1 = sqrt(2)*abs(iC2L2s_1);
			
		if (sign_1 > 0)
		{
			if (Il_ref_1 >= 50) Il_ref_1 = 50;
			else if (Il_ref_1 <= 0) Il_ref_1 = 0;
		}
		else
		{
			if (Il_ref_1 <= -50) Il_ref_1 = -50;
	    	else if (Il_ref_1 >= 0) Il_ref_1 = 0;
		}
		
		// Predictive Control 
		ils_ref_1 = Il_ref_1 * sin(theta); // Resistive Load (fp = 1)
		vlL_1 = v2L1_ref_1 + v2L2_ref_1;
		vl1_ref = vlL_1 + Rl * ils_1 + Ll * (ils_ref_1 - ils_1) * (1/hpwm);
//			vl_ref_1 = 110. * sqrt(2) * sin(theta_1); // Open Loop
		
		
		// Saturator
		if (vl1_ref >= 1.5 * vcr) vl1_ref = 1.5 * vcr;
	    else if (vl1_ref <= -1.5 * vcr) vl1_ref = -1.5 * vcr;

//###################################### FASE 2 ######################################
		if(key){
			P2L1_2 = vC2L1s_ref_2 * iC2L1s_2;
			P2L2_2 = vC2L2s_ref_2 * iC2L2s_2;	
		}
		else{
			P2L1_2 = vC2L1s_ref_2 * iccc;
			P2L2_2 = vC2L2s_ref_2 * iccc;
		}

		Pltot_2 = P2L1_2 + P2L2_2;
		
		if (Pltot_2 > 0) 
			sign_2 = 1;
		else 
			sign_2 = -1;
		
		
		// Simple PI - Fase 2
		//////////////////// 2L1
		vC2L1s_error_2 = sign_2 * (vC2L1s_ref_2 - vC2L1s_2);
		//	vC2L1s_error_2 = (vC2L1s_ref_2 - vC2L1s_2);
		Iv2L1_error_2 = Iv2L1_error_2 + kivl_2 * hpwm * vC2L1s_error_2;
		Pv2L1_error_2 = kpvl_2 * vC2L1s_error_2;
		V2L1_ref_2 = (Iv2L1_error_2 + Pv2L1_error_2);
		
		v2L1_ref_2 = V2L1_ref_2 * sin(theta-p23+angle_g);
		
		//////////////////// 2L2
		vC2L2s_error_2 = sign_2 * (vC2L2s_ref_2 - vC2L2s_2);
		//	vC2L2s_error_2 = (vC2L2s_ref_2 - vC2L2s_2);
		Iv2L2_error_2 = Iv2L2_error_2 + kivl_2 * hpwm * vC2L2s_error_2;
		Pv2L2_error_2 = kpvl_2 * vC2L2s_error_2;
		V2L2_ref_2 = (Iv2L2_error_2 + Pv2L2_error_2);
		
		v2L2_ref_2 = V2L2_ref_2 * sin(theta-p23+angle_g);
		
		
		// FB1 PWM - Fase 2
		vll10_ref_2 =  (v2L1_ref_2 / 2) / vC2L1s_ref_2;
		vlr10_ref_2 = -(v2L1_ref_2 / 2) / vC2L1s_ref_2;
		
		// FB2 PWM - Fase 2	
		vll20_ref_2 =  (v2L2_ref_2 / 2) / vC2L2s_ref_2;
		vlr20_ref_2 = -(v2L2_ref_2 / 2) / vC2L2s_ref_2;
		
		// ######### Load Current Control - Start - Fase 2 ######### 			
		if (abs(V2L1_ref_2) > abs(V2L2_ref_2))
		{
			v2Ls_max_2 = V2L1_ref_2;
			v2Ls_ref_max_2 = 0.9 * vC2L1s_ref_2; // Modulation Index ma = 0.9
		}
		else 
		{
			v2Ls_max_2 = V2L2_ref_2;
			v2Ls_ref_max_2 = 0.9 * vC2L2s_ref_2; // Modulation Index ma = 0.9
		}
		
		// PI
		v2Ls_error_max_2 = -(abs(v2Ls_ref_max_2) - abs(v2Ls_max_2));
		Iil_error_2 = Iil_error_2 + kiil_2 * hpwm * v2Ls_error_max_2;
		Pil_error_2 = kpil_2 * v2Ls_error_max_2;
		Il_ref_2 = sign_2 * (Iil_error_2 + Pil_error_2 + Il_ref0_2);
		//	Il_ref_2 = (Iil_error_2 + Pil_error_2 + Il_ref0_2);
		
		// By the Power
		//	if(abs(iC2L1s_2) > abs(iC2L2s_2)) //P2L1_2/vC2L1s_ref_2 = iC2L1s_2
		//		Il_ref_2 = sqrt(2)*abs(iC2L1s_2);
		//	else				//iC2L1s_2 < iC2L2s_2
		//		Il_ref_2 = sqrt(2)*abs(iC2L2s_2);
				
		//	Saturator
		if (sign_2 > 0)
		{
			if (Il_ref_2 >= 50) Il_ref_2 = 50;
			else if (Il_ref_2 <= 0) Il_ref_2 = 0;
		}
		else
		{
			if (Il_ref_2 <= -50) Il_ref_2 = -50;
			else if (Il_ref_2 >= 0) Il_ref_2 = 0;
		}
		
		// Predictive Control 
		ils_ref_2 = Il_ref_2 * sin(theta-p23); // Resistive Load (fp = 1)
		vlL_2 = v2L1_ref_2 + v2L2_ref_2;
		vl2_ref = vlL_2 + Rl * ils_2 + Ll * (ils_ref_2 - ils_2) * (1/hpwm);
		//	vl_ref_2 = 110. * sqrt(2) * sin(theta_2); // Open Loop
		
		
		// Saturator
		if (vl2_ref >= 1.5 * vcr) vl2_ref = 1.5 * vcr;
		else if (vl2_ref <= -1.5 * vcr) vl2_ref = -1.5 * vcr;

//###################################### FASE 3 ######################################

		if(key){
			P2L1_3 = vC2L1s_ref_3 * iC2L1s_3;
			P2L2_3 = vC2L2s_ref_3 * iC2L2s_3;
		}
		else{
			P2L1_3 = vC2L1s_ref_3 * iccc;
			P2L2_3 = vC2L2s_ref_3 * iccc;		
		}
		

		Pltot_3 = P2L1_3 + P2L2_3;
		
		if (Pltot_3 > 0) 
			sign_3 = 1;
		else 
			sign_3 = -1;
		
		
		// Simple PI - Fase 3
		//////////////////// 2L1
		vC2L1s_error_3 = sign_3 * (vC2L1s_ref_3 - vC2L1s_3);
		//	vC2L1s_error_3 = (vC2L1s_ref_3 - vC2L1s_3);
		Iv2L1_error_3 = Iv2L1_error_3 + kivl_3 * hpwm * vC2L1s_error_3;
		Pv2L1_error_3 = kpvl_3 * vC2L1s_error_3;
		V2L1_ref_3 = (Iv2L1_error_3 + Pv2L1_error_3);
		
		v2L1_ref_3 = V2L1_ref_3 * sin(theta+p23+angle_g);
//		v2L1_ref_3 = 100 * sin(theta);
//		if(v2L1_ref_3 > vC2L1s_ref_3) v2L1_ref_3 = vC2L1s_ref_3;
//		if(v2L1_ref_3 < -vC2L1s_ref_3) v2L1_ref_3 = -vC2L1s_ref_3;		
		
		//////////////////// 2L2
		vC2L2s_error_3 = sign_3 * (vC2L2s_ref_3 - vC2L2s_3);
		//	vC2L2s_error_3 = (vC2L2s_ref_3 - vC2L2s_3);
		Iv2L2_error_3 = Iv2L2_error_3 + kivl_3 * hpwm * vC2L2s_error_3;
		Pv2L2_error_3 = kpvl_3 * vC2L2s_error_3;
		V2L2_ref_3 = (Iv2L2_error_3 + Pv2L2_error_3);
		
		v2L2_ref_3 = V2L2_ref_3 * sin(theta+p23+angle_g);
//		v2L2_ref_3 = 70 * sin(theta);
		
//		if(v2L2_ref_3 > vC2L2s_ref_3) v2L2_ref_3 = vC2L2s_ref_3;
//		if(v2L2_ref_3 < -vC2L2s_ref_3) v2L2_ref_3 = -vC2L2s_ref_3;	
		
		
		// FB1 PWM - Fase 3
		vll10_ref_3 =  (v2L1_ref_3/2) / vC2L1s_ref_3;
		vlr10_ref_3 = -(v2L1_ref_3 / 2) / vC2L1s_ref_3;
		
		// FB2 PWM - Fase 3	
		vll20_ref_3 =  (v2L2_ref_3 / 2) / vC2L2s_ref_3;
		vlr20_ref_3 = -(v2L2_ref_3 / 2) / vC2L2s_ref_3;
		
		// ######### Load Current Control - Start - Fase 3 ######### 			
		if (abs(V2L1_ref_3) > abs(V2L2_ref_3))
		{
			v2Ls_max_3 = V2L1_ref_3;
			v2Ls_ref_max_3 = 0.9 * vC2L1s_ref_3; // Modulation Index ma = 0.9
		}
		else 
		{
			v2Ls_max_3 = V2L2_ref_3;
			v2Ls_ref_max_3 = 0.9 * vC2L2s_ref_3; // Modulation Index ma = 0.9
		}
		
		// PI
		v2Ls_error_max_3 = -(abs(v2Ls_ref_max_3) - abs(v2Ls_max_3));
		Iil_error_3 = Iil_error_3 + kiil_3 * hpwm * v2Ls_error_max_3;
		Pil_error_3 = kpil_3 * v2Ls_error_max_3;
		Il_ref_3 = sign_3 * (Iil_error_3 + Pil_error_3 + Il_ref0_3);
		//	Il_ref_3 = (Iil_error_3 + Pil_error_3 + Il_ref0_3);
		
		// By the Power
		//	if(abs(iC2L1s_3) > abs(iC2L2s_3)) //P2L1_3/vC2L1s_ref_3 = iC2L1s_3
		//		Il_ref_3 = sqrt(2)*abs(iC2L1s_3);
		//	else				//iC2L1s_3 < iC2L2s_3
		//		Il_ref_3 = sqrt(2)*abs(iC2L2s_3);
				
		//	Saturator
		if (sign_3 > 0)
		{
			if (Il_ref_3 >= 50) Il_ref_3 = 50;
			else if (Il_ref_3 <= 0) Il_ref_3 = 0;
		}
		else
		{
			if (Il_ref_3 <= -50) Il_ref_3 = -50;
			else if (Il_ref_3 >= 0) Il_ref_3 = 0;
		}
		
		// Predictive Control 
		ils_ref_3 = Il_ref_3 * sin(theta+p23); // Resistive Load (fp = 1)
		vlL_3 = v2L1_ref_3 + v2L2_ref_3;
		vl3_ref = vlL_3 + Rl * ils_3 + Ll * (ils_ref_3 - ils_3) * (1/hpwm);
		//	vl_ref_3 = 110. * sqrt(2) * sin(theta_3); // Open Loop
		
		
		// Saturator
		if (vl3_ref >= 1.5 * vcr) vl3_ref = 1.5 * vcr;
		else if (vl3_ref <= -1.5 * vcr) vl3_ref = -1.5 * vcr;
		
		//if(flag_switch)  angle_g = -0.371373907; else angle_g = -0.210515901; //2 pra 1
		angle_g =0.0*pi/180.0;
		
//        vl1_ref = 220*sqrt(2)*sin(theta+angle_g);  //tensão de referência na carga fase 1
//        vl2_ref = 220*sqrt(2)*sin(theta - p23+angle_g);  //tensão de referência na carga fase 2
//        vl3_ref = 220*sqrt(2)*sin(theta + p23+angle_g);  //tensão de referência na carga fase 3
            
                 
    	if(vl1_ref > 0){vs0M1 = E/2 - vg1_ref; vs0m1 = vl1_ref -E/2 - vg1_ref;}
		else{vs0M1 = vl1_ref + E/2 - vg1_ref; vs0m1 = -E/2 - vg1_ref;}
		if(vl2_ref > 0){vs0M2 = E/2 - vg2_ref; vs0m2 = vl2_ref -E/2 - vg2_ref;}
		else{vs0M2 = vl2_ref + E/2 - vg2_ref; vs0m2 = -E/2 - vg2_ref;}
		if(vl3_ref > 0){vs0M3 = E/2 - vg3_ref; vs0m3 = vl3_ref -E/2 - vg3_ref;}
		else{vs0M3 = vl3_ref + E/2 - vg3_ref; vs0m3 = -E/2 - vg3_ref;}
                    
    	//max(vs0m1,vs0m2,vs0m3)
		if((vs0m1>=vs0m2)&&(vs0m1>=vs0m3)) vs0m = vs0m1;
		else {if(vs0m2>=vs0m3) vs0m = vs0m2; else vs0m = vs0m3;}
		 
		//min(vs0M1,vs0M2,vs0M3)
		if((vs0M1<=vs0M2)&&(vs0M1<=vs0M3)) vs0M = vs0M1;
		else {if(vs0M2<=vs0M3) vs0M = vs0M2; else vs0M = vs0M3;}
		
		mign = 0.5;
        vgt = mign*vs0M1 + (1.0-mign)*vs0m1;
		
		//vgt = 0.0;
		
		vr1_ref = vg1_ref + vgt;
		vr2_ref = vg2_ref + vgt;
		vr3_ref = vg3_ref + vgt;
		
		vl10_ref = vr1_ref - vl1_ref;
		vl20_ref = vr2_ref - vl2_ref;
		vl30_ref = vr3_ref - vl3_ref;
		
		vr1 = vr1_ref/E;
		vr2 = vr2_ref/E;
		vr3 = vr3_ref/E;
		vl1 = vl10_ref/E;
		vl2 = vl20_ref/E;
		vl3 = vl30_ref/E; 
    	}
		
	//Fim do loop discreto
	
	//PWM: dente de serra normalizada
	if(vserra>=0.5) sinal = -1.0;
	vserra = vserra + sinal*derv*h;
	
	
	//PWM:		
	if(vr1 > vserra) qg1 = 1.0; else qg1 = 0.0;		
	if(vr2 > vserra) qg2 = 1.0; else qg2 = 0.0;
	if(vr3 > vserra) qg3 = 1.0; else qg3 = 0.0;
	if(vl1 > vserra) ql1 = 1.0; else ql1 = 0.0;		
	if(vl2 > vserra) ql2 = 1.0; else ql2 = 0.0;
	if(vl3 > vserra) ql3 = 1.0; else ql3 = 0.0;
	
	if(vll10_ref_1 > vserra) ql2L1_1 = 1; else ql2L1_1 = 0;
	if(vlr10_ref_1 > vserra) qg2L1_1 = 1; else qg2L1_1 = 0;
	if(vll20_ref_1 > vserra) ql2L2_1 = 1; else ql2L2_1 = 0;
	if(vlr20_ref_1 > vserra) qg2L2_1 = 1; else qg2L2_1 = 0;
	
	if(vll10_ref_2 > vserra) ql2L1_2 = 1; else ql2L1_2 = 0;
	if(vlr10_ref_2 > vserra) qg2L1_2 = 1; else qg2L1_2 = 0;
	if(vll20_ref_2 > vserra) ql2L2_2 = 1; else ql2L2_2 = 0;
	if(vlr20_ref_2 > vserra) qg2L2_2 = 1; else qg2L2_2 = 0;
	
	if(vll10_ref_3 > vserra) ql2L1_3 = 1; else ql2L1_3 = 0;
	if(vlr10_ref_3 > vserra) qg2L1_3 = 1; else qg2L1_3 = 0;
	if(vll20_ref_3 > vserra) ql2L2_3 = 1; else ql2L2_3 = 0;
	if(vlr20_ref_3 > vserra) qg2L2_3 = 1; else qg2L2_3 = 0;
	
	if(t<0.1){sg =0; sl=0;}
	
	if(qg1 != qgant) sg = sg + 1;
	qgant = qg1;
	
	if(ql1 != qlant) sg = sg + 1;
	qlant = ql1;
	
	fmg = sg/(0.2);
	
	//Saída	
	out[0] = qg1;
	out[1] = qg2;
	out[2] = qg3;
	out[3] = ql1;
	out[4] = ql2;	
	out[5] = ql3;
	out[6] = qg2L1_1;
	out[7] = ql2L1_1;
	out[8] = qg2L2_1;
	out[9] = ql2L2_1;
	out[10] = qg2L1_2;
	out[11] = ql2L1_2;
	out[12] = qg2L2_2;
	out[13] = ql2L2_2;
	out[14] = qg2L1_3;
	out[15] = ql2L1_3;
	out[16] = qg2L2_3;
	out[17] = ql2L2_3;
	out[18] = flag_switch;
	out[19] = ig1_r;
	out[20] = vg1_ref;
	out[21] = v2L1_ref_1;
	out[22] = v2L2_ref_1;
	out[23] = ils_ref_1;
	out[24] = vl1_ref;
	out[25] = vll10_ref_1;
	out[26] = vlr10_ref_1;
	out[27] = igm;
	out[28] = igm2;
	out[29] = 10;


	}

extern "C" DLLIMPORT void CloseSimUser(const char *szId){
}
