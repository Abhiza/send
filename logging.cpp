//install libsense dari github. 
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// install opencv versi 4
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <array>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <ftdi.h>

#include <eigen3/Eigen/Dense> // library untuk kalkulasi polinomial fitting utnuk perhitungan jalur
#include <eigen3/Eigen/QR> // satu lib

#include <find_polynomial_roots_jenkins_traub.h> //library dari 

//file di linux semua
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h> 
#include <linux/i2c-dev.h>

#include <csvfile.h> // logging data

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

//const char* UART_DEVICE = "/dev/ttyS0"; // Ganti dengan perangkat UART yang sesuai

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace rpoly_plus_plus;

#define CAMERA_WIDTH 640 // resolusi  kamera, coba dibuah, Field of View ditingkatkan
#define CAMERA_HEIGHT 480 //

std::vector<float> x,y,out; // vector buffer untuk sebelum masuk ke vektor bawah ini
std::vector<float> xright,yright,xleft,yleft; // ini vektornya
Mat imgUniversal; // definisi center line untuk destinasi atau arah belok (garis heading kendaraan)
float slope,intercept; // definisi untuk perhitungan dibawah
float middlePoint = CAMERA_WIDTH/2; // titik tengah dari lebar resolusi kamera 

const auto window_name="myCamera"; // nama di GUI opencv

/* Data yang akan diterima dari arduino due*/
int clsobjdet, adjspeed;
int GT = 1;
int lidardistance;
const float gearRatio = 98.7;          // Replace with your gear ratio
const float wheelCircumference = 2.0 * 3.14159 * 0.17; // Replace with your wheel circumference (m)
int outputRPM, rpmMotorTacho;
float speed_kmph;

//MAIN APPS & I2C VARIABLE 
float speed = 0.45;       // setup setpoint speed motor
float steering = 0;         // steering with offset 20 degree (gajadi dulu jadiin 0 dulu)
bool mobiljalan=false;      // status mobil
// on meter float data
float Kdd = 1.00;           // defininya line keeping pure pursuit control, definisi ada jurnal, bisa tanya mas beno
float Lcam = 0.71;
float L = 0.71;
float ld = 2.14;

float heading_angle=0.0f; // definisi awal angle kendaraan

int cvKey = 0; // defini unutk baca kontrol keyboard
bool i2cLatestStartFlag=false; // i2c semua
bool i2cstartFlag=false;
bool i2cSendData=false; // data steering & kecepatan
int i2cFD;
int len; // panjangnya buffer
char *devID = (char*)"/dev/i2c-1"; // settingan port I2C, kemungkinan di jetson bisa beda
int devADDR = 50;    //stm32 slave address
char buffer[6] = {0};
//ACC
int desired_speed;
int min_safe_distance, max_safe_distance;

void i2c_setBuffer(string data) // kolom buffer I2C
{
    //speed = 2; //setting kecepatan disini yang mempengaruhi kecepatan motor stm32 tapi mending yang dibawah (ada komen tempatnya)
    //printf("i2c set buffer\n");
    memset(buffer,0,sizeof(buffer));
    if(data.compare("start")==0) // di bawah adalah code I2C untuk start
    {
        buffer[0] = 0x01; // 
        buffer[1] = 0x00; //
        buffer[2] = 0x00; //
        buffer[3] = 0x00; //
        buffer[4] = 0x01; //
    }
    else if(data.compare("stop")==0) // di bawah adalah code I2C untuk stop
    {
        buffer[0] = 0x01;
        buffer[1] = 0x00;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
    }
    else if(data.compare("set")==0) // di bawah adalah code I2C untuk kecepatan dan belokan
    {
        uint8_t speedDat1 = abs(speed); // bilangan absolute, data 1 dan 2 merupakan kesatuan, 1 digit mewakili satu data, contoh 3,4, dat1= 3, dat2=4 sehingga didapat 3,4
        uint8_t speedDat2 = (uint8_t)((((float)speed)-((float)speedDat1))*100); // data kecepatan tp bentuknya  float
        uint8_t steeringDat1 = abs(steering); //bilangan absolute, sama seperi cara konsep diatas
        uint8_t steeringDat2 = (uint8_t)((((float)steering)-((float)steeringDat1))*100); // data steering tp bentuknya  float
        buffer[0] = 0x02; // confirmasi pengiriman data 
        buffer[1] = speedDat1;
        buffer[2] = speedDat2;
        buffer[3] = steeringDat1;
        buffer[4] = steeringDat2;
    }
    len=5;
}
//ACC
void calculateSafeDistances() {
    if (desired_speed == 100) {
        min_safe_distance = 1000;
        max_safe_distance = 1200;
    } else if (desired_speed == 90) {
        min_safe_distance = 900;
        max_safe_distance = 1000;
    } else if (desired_speed == 80) {
        min_safe_distance = 800;
        max_safe_distance = 900;
    } else if (desired_speed == 70) {
        min_safe_distance = 700;
        max_safe_distance = 800;
    } else if (desired_speed == 60) {
        min_safe_distance = 600;
        max_safe_distance = 700;
    } else if (desired_speed == 50) {
        min_safe_distance = 500;
        max_safe_distance = 600;
    } else if (desired_speed == 40) {
        min_safe_distance = 400;
        max_safe_distance = 500;
    } else if (desired_speed == 30) {
        min_safe_distance = 300;
        max_safe_distance = 400;
    } else {
        min_safe_distance = 300;
        max_safe_distance = 1000;
    }
}

int i2c_write(char *data,int len) // eksekusi nilai buffer data i2c
{
    printf("i2c writing\n");
    char buffReceive[2]; // definisi untuk buffer data nerima data daru stm32
    if(write(i2cFD,data,len)!= len) // membandingkan panjang data, jika tidak sama maka error
    {
        cout << "error write to i2c bus" << endl;
        return 0;
    }
    if(read(i2cFD,buffReceive,1)==1) // membaca data dari stm, untuk konfirmasi panjang buffer
    {
       // cout << to_string(buffer[0]) << endl;
        if(buffReceive[0] == '1') //STM akan mengirim 1(ada di coding stm32 nya), sebagai konfirmasi 
        {
            cout << "i2c write repply success" << endl; 
            return 1;
        }
        else
        {
            cout << "i2c write repply failed" << endl;
            return 0;
        }
    }
    else
        {
            cout << "i2c data write failed" << endl;
            return 0;
        }
}
//ACC
int min_distance() {
    if (desired_speed == 100) {
        return 800;
    } else if (desired_speed == 90) {
        return 700;
    } else if (desired_speed == 80) {
        return 600;
    } else if (desired_speed == 70) {
        return 500;
    } else if (desired_speed == 60) {
        return 400;
    } else if (desired_speed == 50) {
        return 250;
    } else if (desired_speed == 40) {
        return 200;
    } else if (desired_speed == 30) {
        return 150;
    } else {
        return -1; // Kecepatan Tidak Sesuai
    }
}

/*---------------------------------------------------------------------------*/
// threading on C++ (agar program bisa berjalan pararel, menggunakan threading) (ini treadnya)
extern "C" void* myProgram(void* p); // ini nanti di check________________________________________________________________________________________________________________________________________________________________________________________________
extern "C" void* myProgram2(void* p); // ini nanti di check________________________________________________________________________________________________________________________________________________________________________________________________
extern "C" void* myProgram3(void* p);
extern "C" void* myProgram4(void* p);

typedef pthread_t my_Thread;
extern "C" {
  typedef void *(my_Thread_Func)(void *);
}

static int create_thread(my_Thread& t, my_Thread_Func* f, void* p) {
  return pthread_create((pthread_t*)&t, 0, f, p);
}

my_Thread myThread; // tergantung banyak tread yang digunakan diatas
my_Thread myThread2;
my_Thread myThread3;
my_Thread myThread4;

/*---------------------------------------------------------------------------ini sudah intinya ngambil waktu, ada banyak karena program memggunakan waktu delay berbeda*/
//get time on C++ linux
long long int ms,ms2,ms3,ms4;
struct timeval tv;
long int get_millis(void)
{
    gettimeofday(&tv,NULL);
    return ((tv.tv_sec * 1000) + (tv.tv_usec / 1000));
}
/*---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/

/* radian to degree converter */
auto radToDeg = [](std::float_t rad) -> float{ // auto adalah variabel output yang ditentukan oleh sistem
    return ((rad*180.f)/CV_PI);
};

/* degree to radian converter */
auto degToRad = [](std::float_t deg) -> float{
    return ((deg*CV_PI)/180.f);
};

auto purePursuitControl= [](float heading_angle)->float{ // algoritma line keeping 

    float delta = (Lcam*sin(heading_angle*CV_PI/180.0f))/ld; // rumus purePursuit, penjelasan paper man beno
    delta = (asin(delta)*180/CV_PI);
    float alpha = heading_angle-delta;
    
    float steer_angle=radToDeg(atan2(2*L*sin(alpha*CV_PI/180.0f),Kdd));
    steer_angle=round(steer_angle);
    steer_angle=min(20.0f,max(-20.0f,steer_angle));
    return steer_angle;
};

auto polyfit=[](const vector<float>x, // algoritma polynomial fiting dengan Eigen 
                const vector<float>y,
                int order)->vector<float>{
    
    vector<float> coeff;

    if(x.size()==0)
    {
        coeff.push_back(0);
        return coeff;
    }
    else
    {
        assert(x.size()==y.size());
        //assert(x.size() >= order+1);

        //create matrix placeholrder of size n x k, n = num datapoints, k = order polynomial, ex 3 is cubic polynomial
        Eigen::MatrixXf X(x.size(),order+1);
        Eigen::VectorXf Y=Eigen::VectorXf::Map(&y.front(),y.size());
        Eigen::VectorXf result;

        //populate matrix
        for(size_t i=0;i<x.size();++i)
        {
            for(size_t j=0;j<order+1;++j)
            {
                X(i,j) = pow(x.at(i),j);
            }
        }

        //cout << X << endl;

        //solve for linear least square fit
        result = X.householderQr().solve(Y);
        coeff.resize(order+1);
        for(int k=0;k<order+1;k++)
        {
            coeff[k]=result[k];
        }
    }
    return coeff;
};
/*-----------------------------------------------------------------------*/

/*    Lane Keeping Function Helper , mencari slope titik tengah untuk menvari sudut belokan   */
auto getSlope = [](std::vector<float> x, std::vector<float> y) -> std::vector<float>{

    //y = mx+b
    //angle/inclination theta = tan^-1(m)
    float m = ((CAMERA_HEIGHT-y[1]) - (CAMERA_HEIGHT-y[0]))/(x[1]-x[0]);
    float b = (CAMERA_HEIGHT-y[0]) - m*x[0];
    //use atan2 for best result degree on all quadrant
    float theta = (CAMERA_HEIGHT-y[1]) > (CAMERA_HEIGHT-y[0]) ? (atan2((CAMERA_HEIGHT-y[1]) - (CAMERA_HEIGHT-y[0]),x[1]-x[0]))
                                                                :(atan2((CAMERA_HEIGHT-y[0]) - (CAMERA_HEIGHT-y[1]),x[0]-x[1]));
    std::vector<float> out={m,b,theta};
    return out;
};

auto make_points = [](std::vector<float> myCoef,bool position_isRight,float xmin, float xmax,float ymin)-> std::vector<float>{ // menggambar garis tengah
    //y=a+bx+cx^2    
    float x1,x2,y1,y2,discriminant;
    int div = 60; // jumlah titik segment garis, terutama saat berbelok
    int offset=30; // LUPAAAA
    float xTemp1=0.f,xTemp2=0.f; // jngan dirubah!!!!!!!!!!!!!!!!!!
    if(position_isRight)
    {
		xTemp1=CAMERA_WIDTH;
		xTemp2=CAMERA_WIDTH;
	} // sampek sini jgn dirubahnya
    y1=0;  //y position first on bottom
    y2=(CAMERA_HEIGHT/2);  //y position end on middle of frame (reference see ROI)

    vector<double> virtualCoef1,virtualCoef2; // tidak usah diubah, sudah bawaan, sampai line 399
    Eigen::VectorXd xRoots1,xRoots2;

    double realX1=0,realX2=0;

    for(int i=0;i<div-1;i++)
    {
        //xRoots1=find_roots(myCoef,1e-3,i*(y2/div),2,position_isRight);
        //xRoots2=find_roots(myCoef,1e-3,(i+1)*(y2/div),2,position_isRight);
        //if(i>=abs(CAMERA_HEIGHT-ymin) && abs(ymin)!=0)
        //{
        virtualCoef1.clear();virtualCoef2.clear();
        for(int j=0;j<myCoef.size();j++)
        {
            virtualCoef1.push_back((double)myCoef[myCoef.size()-j-1]);
            virtualCoef2.push_back((double)myCoef[myCoef.size()-j-1]);
            if(j==myCoef.size()-1)
            {
                virtualCoef1[j]=virtualCoef1[j]-(i*(y2/div));
                virtualCoef2[j]=virtualCoef2[j]-((i+1)*(y2/div));
            }
        }
        Eigen::VectorXd dCoef1=Eigen::VectorXd::Map(&virtualCoef1.front(),virtualCoef1.size());
        Eigen::VectorXd dCoef2=Eigen::VectorXd::Map(&virtualCoef2.front(),virtualCoef2.size());
        FindPolynomialRootsJenkinsTraub(dCoef1,&xRoots1,NULL);
        FindPolynomialRootsJenkinsTraub(dCoef1,&xRoots2,NULL);
        //const Eigen::IOFormat fmt(2, 1, "\t", " ", "", "", "", "");
        //cout << "--------------------------------------------------------"<<endl;
        //cout << xRoots1.transpose().format(fmt) <<" | " <<xRoots2.transpose().format(fmt)<<endl;
        //cout << "--------------------------------------------------------"<<endl;
        if(position_isRight==true)
        {
            //cout << to_string(xRoots1.size()) << endl;
            //cout << "-------------------" << endl;
            if(xRoots1.size()>1)
            {
                for(int k=0;k<xRoots1.size();k++)
                //for(int k=xRoots1.size()-1;k>=0;k--)
                {
                    if(xRoots1[k]>(double)(xmin-offset) && xRoots1[k]<(double)(xmax+offset))
                    {
                       realX1=xRoots1[k];
                       if(realX1==0)
                       {
						   realX1=CAMERA_WIDTH;
					   }
                       break; 
                    }
                }
            }
            else
            {
				if(xRoots1[0]!=0)
					realX1=xRoots1[0];
				else
					realX1=CAMERA_WIDTH;
            }
            //cout << to_string(xRoots2.size()) << endl;
            //cout << "|-------------------|" << endl;
            if(xRoots2.size()>1)
            {
                //realX2=xRoots2[xRoots2.size()-1];
                for(int k=0;k<xRoots1.size();k++)
                //for(int k=xRoots2.size()-1;k>=0;k--)
                {
                    if(xRoots2[k]>(double)(xmin-offset) && xRoots2[k]<(double)(xmax+offset))
                    {
                       realX2=xRoots2[k];
                       if(realX2==0)
                       {
						   realX2=CAMERA_WIDTH;
						}
                       break; 
                    }
                }
            }
            else
            {
				if(xRoots2[0]!=0)
					realX2=xRoots2[0];
				else
					realX2=CAMERA_WIDTH;
            }
        }
        else
        {
            if(xRoots1.size()>1)
            {
                for(int k=0;k<xRoots1.size();k++)
                {
                    if(xRoots1[k]<(double)(xmax+offset) && xRoots1[k]>(double)(xmin-offset))
                    {
                       realX1=xRoots1[k];
                       break; 
                    }
                }
            }
            else
            {
                realX1=xRoots1[0];
            }

            if(xRoots2.size()>1)
            {
                for(int k=0;k<xRoots2.size();k++)
                {
                    if(xRoots2[k]<(double)(xmax+offset) && xRoots2[k]>(double)(xmin-offset))
                    {
                       realX2=xRoots2[k];
                       break; 
                    }
                }
            }
            else
            {
                realX2=xRoots2[0];
            } 
        }
		//}
        //cout << realX1 << endl;
        //cout << "----------------" << endl;
        //cout << realX2 << endl;
        //cout << "|----------------|" << endl;
        line(imgUniversal,Point((int)realX1,(int)(CAMERA_HEIGHT-(i*(y2/div)))),Point((int)realX2,(int)(CAMERA_HEIGHT-((i+1)*(y2/div)))),Scalar(255,0,0),2,LINE_8,0);
		if(i==0)
		{
			xTemp1=(float)realX1;
		}
        else if(i==div-2)
        {
            xTemp2=(float)realX2;
        }
    }
    //cout << xTemp1 << "|" << y1 << "|" << xTemp2 << "|" << y2 << endl;
    //x1 = std::max((float)-CAMERA_WIDTH,std::min((float)2*CAMERA_WIDTH,((y1-myCoef[0])/myCoef[1])));
    //x2 = std::max((float)-CAMERA_WIDTH,std::min((float)2*CAMERA_WIDTH,((y2-myCoef[0])/myCoef[1])));
    std::vector<float> pos={xTemp1,y1,xTemp2,y2};
    return pos;
};

auto getHeadingAndOffset= [](std::array<std::array<float,4>,2> lane, int len)->std::vector<float>{ // untuk menetukan sudut heading yang digunakan, dinamakan offset
    float left_x2,right_x2,x_offset=0,y_offset=0;
    const float mid = CAMERA_WIDTH/2;
    switch (len)
    {
    case 2:
        /* code */
        left_x2=lane[0][2]; // matrix deteksi garis................. dari fungsi lain
        right_x2=lane[1][2];
        //cout<<"left "<<to_string(left_x2)<<" right "<<to_string(right_x2)<<endl;
        x_offset=((left_x2+right_x2)/2)-mid;
        //x_offset=FIRFilter_Update(&myHeadingFilter,x_offset);
        y_offset=(CAMERA_HEIGHT/2);
        break;
    case 1:
        left_x2=lane[0][0];
        right_x2=lane[0][2];
        x_offset=(right_x2-left_x2);
        y_offset=(CAMERA_HEIGHT/2);
    case 0:
        x_offset = 0;
        y_offset=(CAMERA_HEIGHT/2);
    default:
        break;
    }
    float angle_to_mid_degree=radToDeg(atan(x_offset/y_offset)); // rumus penentuan angle dari titik tengah ke hrading, nanti masuk ke purePursuit
    std::vector<float> out={angle_to_mid_degree,x_offset};
    return out;
};
/*-----------------------------------------------------------------------*/

/*   trackbar on OpenCV To Change Variable Hough Lines Detection (houghlinesp), mengatur batas yang digunakan Houghlines   */ 
int threshVal=100,minLineLen=10,maxLineGap=45,rhoVal=1,thetaVal=1,ISA=0; // ini default tidak usah diatur, bisa diatur di UI saat program jalan, sampai 458

static void on_rhoChange(int, void *)
{
    rhoVal=max(rhoVal,1);
    setTrackbarPos("rho",window_name,rhoVal);
}
static void on_thetaChange(int, void *)
{
    thetaVal=max(thetaVal,1);
    setTrackbarPos("theta",window_name,thetaVal);
}
static void on_threshChange(int, void *)
{
    setTrackbarPos("threshold", window_name, threshVal);
}

static void on_lineLenChange(int, void *)
{
    setTrackbarPos("minLineLength", window_name, minLineLen);
}

static void on_lineGap(int, void *)
{
    setTrackbarPos("maxLineGap", window_name, maxLineGap);
}

static void on_aktivasi(int, void *)
{
    setTrackbarPos("ISA", window_name, ISA);
}


int main(int argc, char * argv[]) try // main program untuk eksekusi semua fungsi
// int main(int argc, char * argv[])
{
        while(mobiljalan==false);-0;/////////////////////////
    if(mobiljalan==true)
    {
    try {
            // Panggil fungsi untuk menghitung jarak aman
            calculateSafeDistances();

            std::cout << "Min Safe Distance: " << min_safe_distance << std::endl;
            std::cout << "Max Safe Distance: " << max_safe_distance << std::endl;

            int min_dist = min_distance();
            if (min_dist != -1) {
                std::cout << "Min Distance for Desired Speed: " << min_dist << std::endl;
            } else {
                std::cout << "Kecepatan Tidak Sesuai" << std::endl;
            }
        } catch (const std::exception& e) {
            // Tangani pengecualian jika ada
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }
    }

    return nullptr;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e) // error pembacaan kamera realsense, catch adalah syntax untuk memberi notif error, nice
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}


//ACC
extern "C" void* myProgram3(void* p) // untuk logging kecepatan sudut yang perintah, output .csv, di stm ada sendiri pake laptop, susah wkwkwkwk -Ifan 2022. sehingga kami (penerus 2023) menggunakan 2 buah encoder (baru dan lama) dan yang lama menjadi input di arduino due dan dikirim (datanya rpmMotorTacho dan speed_kmph) 
{	

}
