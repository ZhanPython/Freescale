//ͨ������ͷ�ɼ������ݽ��д���  �����˲�ȥ��  �õ�������Ϣ �������غ�����ȡ�����������飬������ȣ����ʣ�
/*
������ȡ��ʶ����ߣ�
����ȷ�� 
�������
����
*/
/*

��80*60Ϊ��
��Ե����ȷ��  �ݲ��ñ������䷨  
����ɨ�跨
*/

/*
��һ��ͼ��  ȫ�ֱ����Ƿ��ʼ

*/


#include "common.h"
#include "VCAn_OV7725_EAGLE.h"
#include "MK60_uart.h"
#include "math.h"
#include "PicPro.h"
#include "input.h"


uint8  LeftB[CAMERA_H]={0};   //�������������
uint8 RightB[CAMERA_H]={0};  //�����ұ�������
uint8  linemin=30;//��������������ֵ��Сֵ
uint8  linemax=79;
uint16 right=0;//  
uint16 left=0;
uint8 midline[60];//����������
uint8 diubianR=0;//�ұ߶��߼���
uint8 diubianL=0;//��߶��߼���
uint8 diubianD=0;//���߶��߼���
uint8 Road_state=0;//�ó�ͼ���״̬
 uint8 guaidian=0;//��¼��һ���յ�����
 uint8 Guaidian[60];//�յ�����
uint8 mid9=0;

uint8 jump=0;//��¼�����
uint8  Jump[60];//��¼���������

//��ʼ��������¼
uint8 jump1=0;//��¼�������
uint8  Jump1[60];//��¼�����������

uint8 jump2=0;//��¼�������
uint8  Jump2[60];//��¼�����������

uint8 jump3=0;//��¼�ϰ������
uint8  Jump3[60];//��¼�ϰ����������
  uint8 L=0;
  uint8 R=0;
  
  int R_block=0,L_block=0;

 uint8 Dead=0; //��ֹ��
 uint8 shizi_flag=0; 
 float mid=0;//��Ȩ����ֵ  
 float mid0=0;//��������ֵ  
 float mid1=0;// �ж�����ֵ
 float MID=0;
 uint8 S[]={0};
uint8 S_n=0;
uint8 Circle_flag = 0;
uint8 Island_flag = 0; //��Ϊ0����ʱ��Ϊ���
uint8 Side_Choose = 0; //0Ϊ���м䣬1Ϊƫ����ʻ��2Ϊƫ����ʻ�������Բ��
uint8 SideLoseThre = 20;
uint8 CirOut_Flag = 0;
//uint8 RD_status[40];//���������¼��ÿ��  ��ת ��ת  ά��
extern int Flag_5s ;
extern int mode;
extern uint8 SpeedDown_flag;
extern void OLED_Print_Num(uint8_t x, uint8_t y, uint16_t num);



uint8 DD=0;
int tianji_F=0;
//����ƽ��ֵ�Ĳ�ĺ�
uint8 Curve(uint8 *data,uint8 ave ,uint16 len)
{ uint16 temp=0,curve=0;

 for(uint8 j=0;j<len;j++)
 {
   temp=abs(*(data+j)-ave);
   curve=temp+curve;
   
 }
 
   return curve;
}
//��ƽ��ֵ
uint8 Ave(uint8 *data,uint16 len)
{ uint16 sum=0;
uint8 ave=0;
  for(uint8 i=0;i<len;i++)
  {
    sum=sum+*(data+i); 
  }
  ave=sum/len;
return ave;
}



//ͼ����   ���������������ȷ��   Ԥ����  ���߼�¼
void   FirstPic( uint8 *imgaddr,uint32 imgsize )  //���� ͼ������  
{                                                         //��һ��ͼ����



  uint8 LF=0,RF=0; //����������¼���
  uint8 i=0,j=0;
    for(uint8 t=0;t<60;t++){RightB[t]=0;}    
    for(uint8 t=0;t<60;t++){LeftB[t]=0;}
    for(uint8 t=0;t<60;t++){midline[t]=0;}
  
    //ǰ10�б��������
  imgaddr=imgaddr+imgsize-1;


  if(*(imgaddr-40)==0)  //����Ϊ��·
  {
    Side_Choose = 0;
    for(i=0;i<40;i++)
    {  
       if(*(imgaddr+i-40)==0&&*(imgaddr+i-40+1)==255&&RF==0)  //ȷ���ұ߽߱�
       {
         RightB[0]=40+i;
         RF=1;
       }
       if(*(imgaddr-i-40+1)==0&&*(imgaddr-i-40)==255&&LF==0) //ȷ����߽߱�
       {
         LeftB[0]=39-i;
         LF=1; 
       }
    }
    if(RF==0&&LF==0) //����ȫ����
    {   
      RightB[0]=79;LeftB[0]=0;
    }
    else if (RF==1&&LF==0)//��߶���  ֱ�Ӹ���
    {   
      LeftB[0]=0;
    }
    else if (RF==0&&LF==1)//�ұ߶���  ֱ�Ӹ���
    {
      RightB[0]=79;
    }
    midline[0]=(RightB[0]+LeftB[0])/2; //uart_putchar(VCAN_PORT,midline[j]);
    LF=0;RF=0;
   }
   else if (*(imgaddr-40)==255)  //���߷ǵ�·
   {
     if (Side_Choose == 0)
     {
       for(i=0;i<40;i++)
       {  
         if(*(imgaddr+i-39)==0&&RF==0)    //����Ѱ�ҵ���߽������
         {
           LeftB[0]=40+i;RightB[0]=79;
           RF=1;
         }
         if(*(imgaddr-i-40)==0&&LF==0)    //����Ѱ�ҵ��ұ߽������
         {
           RightB[0]=39-i;LeftB[0]=0;
           LF=1; 
         }
       }
     }
     else if (Side_Choose == 1)
     {
       for (i=0;i<80;i++)
       {
         if (*(imgaddr+i-79)==0&&LF==0)
         {
           LeftB[0] = i;
           LF = 1;
         }
         if (*(imgaddr+i-79)==255&&LF==1&&RF==0)
         {
           RightB[0] = i; 
           RF = 1;
         }
       }
     }
     else if (Side_Choose == 2)
     {
       for (i=0;i<80;i++)
       {
         if (*(imgaddr-i)==0&&RF==0)
         {
           RightB[0] = 79-i;
           RF = 1;
         }
         if (*(imgaddr-i)==255&&RF==1&&LF==0)
         {
           LeftB[0] = 80-i; 
           LF = 1;
         }
       }
     }
      midline[0]=(RightB[0]+LeftB[0])/2; //uart_putchar(VCAN_PORT,midline[j]);
      LF=0;RF=0;
   }
    imgaddr=imgaddr-80;
  //1-58
   for(j=1;j<60;j++)   
   {     
      imgaddr=imgaddr-80;
      for(i=0;i<((RightB[j-1]-LeftB[j-1]/2)+15);i++)
      {  
        if(*(imgaddr+1+midline[j-1]+i)==255&&RF==0&&midline[j-1]+i<80)//&&Island_flag==0)  //Circle �޸� 
        { 
          RightB[j]=midline[j-1]+i;  
          RF=1;  
        }    
        if(*(imgaddr+1+midline[j-1]-i)==255&&LF==0&&(midline[j-1]-i>0||midline[j-1]-i==0))//&&Island_flag==0) // Circle �޸�
        { 
         LeftB[j]=midline[j-1]-i;
         LF=1;  
        }
        
        if ((RightB[j-1]-LeftB[j-1]>=30)&&LF==1&&RF==1&&i==0)    // Circle �޸� ���Ļ����ж�  Island_flag==0&&  alteration if (LeftB[j]==RightB[j]&&RightB[j]==1)
        {
          if (LeftB[j]+RightB[j]<80) {RightB[j]=RightB[j-1];LeftB[j]=(2*RightB[j]+1*midline[j-1])/3;}  //ƫ���ұ���ʻ��������߽�
          else if (LeftB[j]+RightB[j]>=80) {LeftB[j]=LeftB[j-1]; RightB[j]=(1*midline[j-1]+2*LeftB[j])/3;}  //ƫ�������ʻ�������ұ߽�
          Island_flag = j;
        } 
       }
       if(RF==0&&LF==0) //����ȫ����
       { 
          LeftB[j]=LeftB[j-1];
          RightB[j]=RightB[j-1];
       }
       else if(RF==1&&LF==0)//��߶���
       {  
         LeftB[j]=0;
       }
       else if (RF==0&&LF==1)//�ұ߶���  ֱ�Ӹ���
       {
         RightB[j]=79;
       }
       midline[j]=(RightB[j]+LeftB[j])/2; 
       LF=0;RF=0;
  }
}

 //����ʮ�д���
 
uint8 b=0;

//ֱ��33 ��ת22 ��ת11 ʮ��55Сs 44 ��s 66 ���������77 �����88  �ϰ�99

//ȷ��ֱ���������Сs��ʮ�֣�ʮ�ֲ���
void RouD()
{     
 Road_state = 0;
 guaidian=0;
 Dead=0;
 jump=0;
 diubianR=0;
 diubianL=0;
 diubianD=0;
 shizi_flag=0;
 uint8  block_flag=0;
 
jump1=0;//��¼�������
jump2=0;//��¼�������
jump3=0;//��¼�ϰ������
DD=0;

 
  for(int i=0;i<60;i++)
  { 
   Jump[i]=0;
   Jump1[i]=0;
   Jump2[i]=0;
   Jump3[i]=0;
   Guaidian[i]=0;
  }
 
  //uint8 ten_n;
  //uint8 ten[20];//shizi
  //uint8 RightW=0;
  //uint8 LeftW=0;
  uint8 LeftN=0;//��¼���ҹյ����
  uint8 RightN=0;
  //uint8 Cross=0;
  uint8 Road[50]={0};  //��¼����ÿ������ 0ֱ��  1 ����  2 ����
  //uart_putchar(VCAN_PORT,170);
        
   for(uint8 k=0;k<60;k++)//Ѱ�ҹյ� 48��������Ϊ����Ч��  ȫ���У���¼��������  
   { 
     if(LeftB[k]==RightB[k]&&RightB[k]==midline[k]&&Dead==0)
     { 
       Dead=k;
     }
   }
   if (Dead==0){Dead=60;}
   //if (Island_flag>=1&&Island_flag<=7) {Dead = 0;}  //�йؽ�ֹ�߽��circle���޸�  20170628  ɾ��
   //if (Side_Choose!=0&&(RightB[58]-LeftB[58])<25) {Dead = 0;}
   
   
   for(uint8 k=1;k<Dead;k++)//Ѱ�ҹյ� 48��������Ϊ����Ч��  ȫ���У���¼�������� 
   { 
 
     if(LeftB[k]-LeftB[k-1]>0)
     { 
       if( RightB[k]-RightB[k-1]<0 )         {Road[k]=0;}
       else if(RightB[k]-RightB[k-1]==0)  {Road[k]=Road[k-1];}
       else if(RightB[k]-RightB[k-1]>0)   {Road[k]=2;}
     }
     else if(LeftB[k]-LeftB[k-1]==0)
     { 
       if(RightB[k]-RightB[k-1]<0)          {Road[k]=Road[k-1];}
       else if(RightB[k]-RightB[k-1]==0)   {Road[k]=Road[k-1];}
       else if(RightB[k]-RightB[k-1]>0)    {Road[k]=2;}
       //Circle�޸�20170618  ��߶������ұ�������˵����ƫ���·�Ҳ࣬�����ҹ�
       if (LeftB[k]==0&&k<=SideLoseThre&&(RightB[k]-RightB[k-1]>0)&&RightB[k]>55&&Island_flag>=1)  {Road[k]=2; Circle_flag=1;}
     }
     else if(LeftB[k]-LeftB[k-1]<0)
     { 
       if (RightB[k]-RightB[k-1]<0)         {Road[k]=1;}
       else if(RightB[k]-RightB[k-1]==0)   {Road[k]=1;}
       else if(RightB[k]-RightB[k-1]>0)    
       {
         if (RightB[k]+LeftB[k]<80) {Road[k]=2; Circle_flag=1;}  //������λ��ͼ����࣬˵����ƫ����ʻ�������ҹ�
         else if (RightB[k]+LeftB[k]>=80) {Road[k]=1; Circle_flag=1;}  //������λ��ͼ���Ҳ࣬˵����ƫ����ʻ���������
         
       }   ////Circle�޸�20170615  ����Բ�Ͷ���Ϊ���ȴ������ʻ
       
       //Circle�޸�20170618  �ұ߶��������������˵����ƫ���·��࣬�������
       if (RightB[k]==79&&k<=SideLoseThre&&LeftB[k]<25&&Island_flag>=1) {Road[k]=1; Circle_flag=1;}
     }
     
     //if (CirOut_Flag==0&&k<=20&&(RightB[k]-LeftB[k])>40) {CirOut_Flag==k;}
     //if (CirOut_Flag!=0&&(k>=CirOut_Flag+10)&&(RightB[k]-LeftB[k])<20) {CirOut_Flag==2;} //ûʲô����
   }
   
   if (Island_flag>=5) {Circle_flag = 1;}  //��ֹ��ʧԲ�� ����20170628 
   if ((Circle_flag == 1 || Road_state==RD_circle)&& Island_flag<=40 )
      { SpeedDown_flag = 1;}
  

   //uint8 *data;
   //data=&Road[4];
   //uart_putbuff(VCAN_PORT,data,46);
   //����������������
    for (uint8 i=1;i<Dead;i++)
    {  
      if(Road[i]==2)
      {  
          Guaidian[RightN]=i;//��¼�յ������
          RightN++;  
      }
      else if(Road[i]==1)
      {  
         Guaidian[LeftN]=i;//��¼�յ������
         LeftN++;
      }
    }
   
   for(uint8 i=1;i<=Dead;i++)
   {
     if(abs(RightB[i]-RightB[i-1])>6&&((LeftB[i-1]<=25&&RightB[i-1]==79)||(RightB[i-1]>=55&&LeftB[i-1]==0))){Jump[jump]=i;jump++;}
     else if(abs(LeftB[i]-LeftB[i-1])>6&&((LeftB[i-1]<=25&&RightB[i-1]==79)||(RightB[i-1]>=55&&LeftB[i-1]==0))){Jump[jump]=i;jump++;}
   }
   for(uint8 i=1;i<35;i++)
   {
     if(abs(RightB[i]-RightB[i-1])>8&&(RightB[i-1]!=79||LeftB[i-1]!=0))  {Jump2[jump2]=i;jump2++;}
     if(abs(LeftB[i]-LeftB[i-1])>8&&(RightB[i-1]!=79||LeftB[i-1]!=0))     {Jump1[jump1]=i;jump1++;}
   
   }
   for(uint8 i=1;i<55&&i<Dead;i++)
   {
     if(abs(RightB[i]-RightB[i-1])>8&&LeftB[i]!=0&&LeftB[i-1]!=0)     {    R_block=1;    Jump3[jump3]=i;jump3++;}
     else if(abs(LeftB[i]-LeftB[i-1])>8&&RightB[i]!=79&&RightB[i-1]!=79)      {  L_block=1;  Jump3[jump3]=i;jump3++;}
     else if (abs(LeftB[i]-LeftB[i-1])>8&&abs(RightB[i]-RightB[i-1])>8)  {    R_block=1; L_block=1; Jump3[jump3]=i;jump3++;}
   }
   for(uint8 i=0;i<15;i++)
   { if(Jump3[i+1]-Jump3[i]>=15) {block_flag=Jump3[i+1];}}
   
   if(Jump3[0]<=35&&Jump3[0]>=10&&(RightB[0]-LeftB[0])<=50){block_flag=Jump3[0];} 
   b=block_flag;   
 
   //�յ������
   guaidian=Guaidian[0];//�յ������һ��Ϊ�յ�
   if ( Island_flag > 5 ){ guaidian = Island_flag; } //Բ���յ㲹�� 20170628
 
   for(uint8 i=0;i<Dead;i++)
   {
     if(RightB[i]==79) diubianR++;
     if(LeftB[i]==0)   diubianL++;
     if(RightB[i]==79&&LeftB[i]==0){if(DD==0){DD=i;} diubianD++;}
   }

     //�жϼ�¼������Ϣ�� ���Ҳ���
  uint8 zuo=0;
  uint8 you=0;
  for(int i=0;i<50;i++)
  {    
    if(Road[i]==1&&RightB[i]==79&&shizi_flag==0)
    {shizi_flag=1; zuo=i;}
    if(Road[i]==2&&LeftB[i]==0&&shizi_flag==0)
    {shizi_flag=2; you=i;}
    //if(Road[1]==1&&RightB[i]<=78)
    //{shizi_flag=1; zuo = i;}
    //Circle�޸�20170615
  }
   L=0;
   R=0;
  
  if(shizi_flag==1){ for(int i=zuo-4;i<zuo+4;i++) {if(LeftB[i]==LeftB[zuo]) L++;  }} 
  if(shizi_flag==2){ for(int i=you-4;i<you+4;i++) {if(RightB[i]==RightB[you]) R++;  }} 

  if((diubianD>3&&DD>6)||(shizi_flag==1&&(diubianR>35||diubianD>=20)&&jump>0)||(shizi_flag==2&&(diubianL>35||diubianD>=20)&&jump>0))
  {
    Road_state=RD_shizi; 
    if(shizi_flag==1)
    {  
       int p=0;
       for(int n=zuo;n<Dead&&RightB[n]==79;n++)  
      {
        if(p%2==0){midline[n]=midline[zuo]+p/2;}
        else {midline[n]=midline[n-1];}
        p++;
      }
      Dead=zuo+p;
    }
    else if(shizi_flag==2)
    {  
      int p=0; 
      for(int n=you;n<Dead&&LeftB[n]==0;n++) 
      {
        if(p%2==0){midline[n]=midline[zuo]-p/2;}
        else {midline[n]=midline[n-1];}
        p++;
      }
      Dead=you+p;
      
    }
  }//PASS
  else if (Circle_flag==1 && guaidian<=40)  //1000��ʱ����30
  {
    Road_state = RD_circle;
    for (int i=guaidian;i<=Island_flag;i++)   //�ɵ������Ż�20170621
    {
     if (Road[guaidian]==1)  //(Side_Choose==0||Side_Choose==1)&&
     {
       Side_Choose = 1;
       midline[i]=LeftB[i]+(midline[Island_flag-1]-LeftB[Island_flag])/4;//(RightB[Island_flag+5]-LeftB[Island_flag+5])*0.3;
     }
     if (Road[guaidian]==2)  //(Side_Choose==0||Side_Choose==2)&&
     {
       Side_Choose = 2;
       midline[i]=RightB[i]-(RightB[Island_flag]-midline[Island_flag-1])/4;
     }
    }
    
    for (int i=1;i<guaidian;i++)
    {
      if ((Side_Choose==0||Side_Choose==1)&&Road[guaidian]==1)
      {
        midline[i]=midline[i]/2;
      }
      else if ((Side_Choose==0||Side_Choose==2)&&Road[guaidian]==2) 
      {
        midline[i]=midline[i]*3/2;
      }
    }
    Island_flag=0;
    Circle_flag=0;
  }
/*****************7.14 �޸�***********����ΪԤʮ��*****************/
  else if(block_flag>=1&&Dead==60)  //·��Road_state!=RD_shizi&&
  {
     Road_state=RD_block; Dead=block_flag;
  }
  else if((shizi_flag==1&&diubianR>45&&L<=1)||shizi_flag==2&&diubianL>45&&R<=1 )//Ԥʮ���б�
  {
   Road_state=RD_Pshizi;
   if(shizi_flag==1)
   {  int p=0;
     for(int n=zuo;n<Dead&&RightB[n]==79;n++)
     { 
       if(p%2==0)
       {
         midline[n]=midline[zuo]+p/2;
       }
       else 
       {
         midline[n]=midline[n-1];
       }
       p++;
      }
      Dead=zuo+p;
   }
   else if(shizi_flag==2)
   {  
     int p=0; 
     for(int n=you;n<Dead&&LeftB[n]==0;n++) 
     {
       if(p%2==0)
       {
         midline[n]=midline[zuo]-p/2;
       }
     else 
     {
       midline[n]=midline[n-1];
     }
     p++;
     }
     Dead=you+p;
   }
  }
 

 else if(diubianR>(Dead-5))//��������77   ������Ŀ�� ��׼ȷ 
   {//uart_putchar(VCAN_PORT,1);
     if(Dead>55)  {Road_state=RD_chuyouwan;}
    /* else if(Dead>48) {Road_state=RD_youzhuan;}*/
     
     else if(Dead<=40){Road_state=RD_youwan;}
     else if(Dead>40&&Dead<=54) {Road_state=RD_youzhuan;}
     OLED_Print_Num(88,2,Road_state); 
       //����
//       for(uint8 n=5;n<40&&midline[n]<80;n++)  
//       {   
//       midline[n]=LeftB[n]+midline[5]-LeftB[5];
//       Dead=n;
//       }
//       Dead=Dead+1;
//       
   }
  else if(diubianL>(Dead-5))//��������88
  {  if(Dead>45)  {Road_state=RD_chuzuowan;}
 //uart_putchar(VCAN_PORT,2);
      else if(Dead<=44) Road_state=RD_zuowan;
      
       //����
//       for(uint8 n=5;n<40&&midline[n]<80;n++)
//       {   
//       midline[n]=RightB[n]-(RightB[5]-midline[5]);
//       Dead=n;
//       }
//         Dead=Dead+1;
       
  }
  

 
   else if(RightN>5&&LeftN==0&&guaidian<=16)  //ֻ����һ����ת������
   { // uart_putchar(VCAN_PORT,3);//��ת22�����޸������ߣ��յ��Ժ󲻴���
      Road_state=RD_youzhuan;
      
         //buxian
//      for(uint8 i=guaidian+1;i<Dead;i++)
//      {
//        RightB[i]=LeftB[i]+RightB[guaidian]-LeftB[guaidian];
//        midline[i]=(RightB[i]+LeftB[i])/2;
//      }
   }
   
  
    else if(RightN>5&&LeftN==0&&guaidian<=35&&guaidian>=16)  //ֻ����һ����ת������
   { // uart_putchar(VCAN_PORT,3);//��ת22�����޸������ߣ��յ��Ժ󲻴���
      Road_state=RD_youzhuan1;
      
         //buxian
//      for(uint8 i=guaidian+1;i<Dead;i++)
//      {
//        RightB[i]=LeftB[i]+RightB[guaidian]-LeftB[guaidian];
//        midline[i]=(RightB[i]+LeftB[i])/2;
//      }
   }
   
  
      else if(RightN==0&&LeftN>5&&guaidian<=35&&guaidian>=12)
   {  //uart_putchar(VCAN_PORT,4);//��ת11�����޸������ߣ��յ��Ժ󲻴���
         Road_state=RD_zuozhuan1;
         
         //buxian
//          for(uint8 i=guaidian+1;i<Dead;i++)
//      {
//       LeftB[i]=RightB[i]-(RightB[guaidian]-LeftB[guaidian]);
//        midline[i]=(RightB[i]+LeftB[i])/2;
//      }
   }
   
   else if(Road_state!=RD_shizi&&jump1>0&&jump2>0&&diubianD<=3&&(diubianL<=25||diubianR<=25))//��ʼ���б�  7/20
   //{Road_state=RD_qishixian;tianji_F=1;}
   { 
     Road_state=RD_qishixian;
     if(Flag_5s ==1)tianji_F=1;
   }
   else //�յ�>40�򲻴��ڹյ����ΪΪֱ��
   {  //uart_putchar(VCAN_PORT,5);//ֱ��33      
     Road_state=RD_zhidao;
   }
     
   OLED_Print_Num(88,2,Road_state);
              
}

//�����ߣ�����·�ߣ���ȡ  ƫ�����



// ��ȡ�ο�������  ֱ����ƽ��
void MID_Get()
{   
  uint32 sum=0;//ǰ�κ�
  uint32 sum1=0;//��κ�
  uint8 Dead1=0,Dead2=0;
  mid0=0;
  mid1=0;
  Dead1=(int)(Dead/3*2);
  Dead2=Dead-Dead1;
  
  for(uint8 i=0;i<Dead1;i++)
  {
    sum=sum+midline[i];
  }
  mid0=sum/Dead1;
  for(uint8 j=Dead1;j<Dead;j++)  
  {
    sum1=sum1+midline[j];
  }
  mid1=sum1/Dead2;
  
//  mid=0.75*mid0+0.25*mid1;
  // mid=0.7*mid0+0.3*mid1;
  //Ȩ�ط�ģʽ
  if(mode==mode_1)      {mid=0.75*mid0+0.25*mid1;} 
  else if(mode==mode_2) {mid=0.7*mid0+0.3*mid1;} 
  else if(mode==mode_3) {mid=0.7*mid0+0.3*mid1;} 
  else if(mode==mode_4) {mid=0.65*mid0+0.35*mid1;}
  else if(mode==mode_5) {mid=0.65*mid0+0.35*mid1;}
    else if(mode==mode_5) {mid=0.65*mid0+0.35*mid1;}
   else if(mode==mode_6) {mid=0.62*mid0+0.38*mid1;}
   else if(mode==mode_9) {mid=0.8*mid0+0.2*mid1;}
     else  {mid=0.7*mid0+0.3*mid1;} 

}


