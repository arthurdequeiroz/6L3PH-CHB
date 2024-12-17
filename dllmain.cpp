#include <windows.h>
#include <cmath>
#include "dll.h"


//constantes
double h=0.000001, w=376.991118431, p23=2.09439510239, hpwm = 0.0001, pi=3.14159265;
//Entradas
double vlm=0.0, E=0.0, teta=0.0, vgm=0.0, eg1=0.0, delta=0.0, tf=0.0, egm=0.0, ig3=0.0, ig2=0.0, ig1=0.0, eg2=0.0, eg3=0.0, vc=0.0;
//vari�veis da estrat�gia de PWM
double ts = 0.0, derv = 0.0, vserra=0.0, sinal=0.0, migt=0.5, vg1_ref=0.0, vg2_ref=0.0, vg3_ref=0.0, vl1_ref=0.0, vl2_ref=0.0, vl3_ref=0.0, vgmax=0.0, 
vgmin=0.0, vgt_max=0.0, vgt_min=0.0, vgt=0.0, vr1=0.0, vr2=0.0, vr3=0.0, vl1=0.0, vl2=0.0, vl3=0.0, ang=0.0, vr1_ref=0.0, vr2_ref=0.0, vr3_ref=0.0,
vl10_ref=0.0, vl20_ref=0.0, vl30_ref=0.0; 
//vari�veis do controle de barramento
double kic = 10.0;
double kpc = 0.25;
double xic=0.0, xica=0.0, erB=0.0, erBa=0.0, vcr=0.0; 
//vari�veis do controle de corrente
double kii = 1000.0;
double kpi = 1.0;
double ig1_r=0.0, ig2_r=0.0, ig3_r=0.0, igm=0.0, er1=0.0, er2=0.0, er1a=0.0, er2a=0.0, xaa1=0.0,xa1=0.0,xb1=0.0,xaa2=0.0,xa2=0.0,xb2=0.0;
double F1 = cos(hpwm*w), F2 = sin(hpwm*w)/w, F3 = 2.0*kii*sin(hpwm*w)/w, 
F4=-w*sin(w*hpwm), F5=cos(w*hpwm), F6=(cos(hpwm*w)-1.0)*2.0*kii;
//estados das chaves
double qg1=0.0, qg2=0.0, qg3=0.0, ql1=0.0, ql2=0.0, ql3=0.0;
//frequ�ncia m�dia
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
//transit�rio de carga
double flag_switch = 0.0;
//gambiarras
double angle_g;
//ITH
double vs0m1, vs0M1, vs0m2, vs0M2, vs0m3, vs0M3, vs0m, vs0M, mign;


	
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
    
    derv = 2/hpwm;
    
    F1 = cos(hpwm*w); F2 = sin(hpwm*w)/w; F3 = 2.0*kii*sin(hpwm*w)/w; 
	F4=-w*sin(w*hpwm); F5=cos(w*hpwm); F6=(cos(hpwm*w)-1.0)*2.0*kii;
    
	//loop discreto
	if(ts <= t){
		ts = ts + hpwm;
		vserra = -0.5;
		sinal = 1.0;
		
	//Transit�rio de carga
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
	
	Pdf = C1* X1af + C2*X2af + D1*Pd; //Saida do Filtro

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
	
	// ===========================================================
		//Controle de corrente
		//igm = 1.0*30.0*sqrt(2);
		
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
		
		if(vg1_ref > 2*E) vg1_ref = 2*E;
		if(vg1_ref < -2*E) vg1_ref = -2*E;
		if(vg2_ref > 2*E) vg2_ref = 2*E;
		if(vg2_ref < -2*E) vg2_ref = -2*E;
		if(vg3_ref > 2*E) vg3_ref = 2*E;
		if(vg3_ref < -2*E) vg3_ref = -2*E;
		
		
		//vg1_ref = 150*sqrt(2)*sin(theta+angle_g);  //tens�o de refer�ncia na carga fase 1
        //vg2_ref = 150*sqrt(2)*sin(theta - p23+angle_g);  //tens�o de refer�ncia na carga fase 2
        //vg3_ref = 150*sqrt(2)*sin(theta + p23+angle_g);  //tens�o de refer�ncia na carga fase 3
		
		//if(flag_switch)  angle_g = -0.371373907; else angle_g = -0.210515901; //2 pra 1
		angle_g =0.0*pi/180.0;
		
        vl1_ref = 220*sqrt(2)*sin(theta+angle_g);  //tens�o de refer�ncia na carga fase 1
        vl2_ref = 220*sqrt(2)*sin(theta - p23+angle_g);  //tens�o de refer�ncia na carga fase 2
        vl3_ref = 220*sqrt(2)*sin(theta + p23+angle_g);  //tens�o de refer�ncia na carga fase 3
            
                 
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
	
	if(t<0.1){sg =0; sl=0;}
	
	if(qg1 != qgant) sg = sg + 1;
	qgant = qg1;
	
	if(ql1 != qlant) sg = sg + 1;
	qlant = ql1;
	
	fmg = sg/(0.2);
	
	//Saída	
	out[0]  = qg1;
	out[1]  = qg2;
	out[2]  = qg3;
	out[3]  = ql1;
	out[4]  = ql2;	
	out[5]  = ql3;	
	out[6]  = flag_switch;
	out[7]  = fmg;
	out[8] = fml;
	out[9] = wfg;
	out[10] = vl1_ref;
	out[11] = vg1_ref;
	out[12] = ig1_r;
	out[13] = vl1;



	}

extern "C" DLLIMPORT void CloseSimUser(const char *szId){
}
