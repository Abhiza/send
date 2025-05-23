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
#include <algorithm>
//#include <ftdi.h>

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
float speedq = 0.45;
float steering = 0;         // steering with offset 20 degree (gajadi dulu jadiin 0 dulu)
float brake = 0;
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
char buffer[8] = {0};
//ACC
int desired_speed;
int min_safe_distance, max_safe_distance;

// tele
int steeringtele = 20;
int inputtele = 0;
int braketele = 0;

bool press = false;
bool sButtonPress = false;

float jarak_depth; 
int brake_signal;

// fuzzy
// Define the membership function class
class MembershipFunction {
public:
    virtual double evaluate(double value) const = 0;
};

// Define triangular membership function
class TriangularMembershipFunction : public MembershipFunction {
    double a, b, c;

public:
    TriangularMembershipFunction(double a, double b, double c) : a(a), b(b), c(c) {}

    double evaluate(double value) const override {
        if (value <= a || value >= c) return 0.0;
        if (value == b) return 1.0;
        if (value > a && value < b) return (value - a) / (b - a);
        return (c - value) / (c - b);
    }
};

// Define trapezoidal membership function
class TrapezoidalMembershipFunction : public MembershipFunction {
    double a, b, c, d;

public:
    TrapezoidalMembershipFunction(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}

    double evaluate(double value) const override {
        if (value <= a || value >= d) return 0.0;
        if (value > a && value <= b) return (value - a) / (b - a);
        if (value > b && value <= c) return 1.0;
        if (value > c && value < d) return (d - value) / (d - c);
        return 0.0;
    }
};

// Define the fuzzy rule class
class FuzzyRule {
    const MembershipFunction &input1, &input2;
    double outputValue;

public:
    FuzzyRule(const MembershipFunction &input1, const MembershipFunction &input2, double outputValue)
        : input1(input1), input2(input2), outputValue(outputValue) {}

    double evaluate(double value1, double value2) const {
        return std::min(input1.evaluate(value1), input2.evaluate(value2)) * outputValue;
    }

    double getOutputValue() const {
        return outputValue;
    }
};

// Define the fuzzy logic controller class
class FuzzyController {
    std::vector<FuzzyRule> rules;

public:
    void addRule(const FuzzyRule &rule) {
        rules.push_back(rule);
    }

    double evaluate(double input1, double input2) const {
        double numerator = 0.0;
        double denominator = 0.0;

        for (const auto &rule : rules) {
            double ruleEval = rule.evaluate(input1, input2);
            numerator += ruleEval;
            denominator += ruleEval > 0 ? ruleEval / rule.getOutputValue() : 0;
        }

        return denominator == 0 ? 0 : numerator / denominator;
    }
};


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
        buffer[4] = 0x00; //
        buffer[5] = 0x00; //
        buffer[6] = 0x01; //
    }
    else if(data.compare("stop")==0) // di bawah adalah code I2C untuk stop
    {
        buffer[0] = 0x01;
        buffer[1] = 0x00;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = 0x00;
    }
    else if(data.compare("set")==0) // di bawah adalah code I2C untuk kecepatan dan belokan
    {
        uint8_t speedDat1 = abs(speed); // bilangan absolute, data 1 dan 2 merupakan kesatuan, 1 digit mewakili satu data, contoh 3,4, dat1= 3, dat2=4 sehingga didapat 3,4
        uint8_t speedDat2 = (uint8_t)((((float)speed)-((float)speedDat1))*100); // data kecepatan tp bentuknya  float
        uint8_t steeringDat1 = abs(steering); //bilangan absolute, sama seperi cara konsep diatas
        uint8_t steeringDat2 = (uint8_t)((((float)steering)-((float)steeringDat1))*100); // data steering tp bentuknya  float
        uint8_t brakeDat1 = abs(brake); //bilangan absolute, sama seperi cara konsep diatas
        uint8_t brakeDat2 = (uint8_t)((((float)steering)-((float)steeringDat1))*100); // data steering tp bentuknya  float
  
        buffer[0] = 0x02; // confirmasi pengiriman data 
        buffer[1] = speedDat1;
        buffer[2] = speedDat2;
        buffer[3] = steeringDat1;
        buffer[4] = steeringDat2;
        buffer[5] = brakeDat1;
        buffer[6] = brakeDat2;
    }
    len=7;
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
    cout << "len = "<<len << endl;    
   
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
//extern "C" void* myProgram2(void* p); // ini nanti di check________________________________________________________________________________________________________________________________________________________________________________________________
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
// my_Thread myThread2;
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
    static bool first = true;
    //create configuration
    rs2::config cfg;
    rs2_format myFormat=RS2_FORMAT_BGR8;
    cout << "use color format -> " << rs2_format_to_string(myFormat) <<endl;
    cfg.enable_stream(RS2_STREAM_COLOR,0,CAMERA_WIDTH,CAMERA_HEIGHT,myFormat,30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;

    // configure and start the pipeline
    rs2::pipeline_profile selection = pipe.start(cfg);

    rs2::device selected_dev = selection.get_device();

    auto rgbSensor = selected_dev.first<rs2::color_sensor>();


    const float min_distance_cm = 30.0; // Minimum distance in cm

    // Define the coordinates of the 148 points
    std::vector<std::pair<int, int>> coordinates = {


{231,478}, {234,472}, {239,464}, {242,459}, {245,451}, {249,441}, {255,430}, 
{259,422}, {266,406}, {273,396}, {279,388}, {281,378}, {284,374}, {289,367}, 
{293,360}, {296,352}, {299,346}, {303,339}, {305,336}, {307,330}, {310,324}, 
{314,319}, {315,313}, {439,470}, {436,466}, {434,462}, {433,458}, {428,450}, 
{426,445}, {423,440}, {421,433}, {421,433}, {418,428}, {415,420}, {412,416}, 
{407,408}, {404,403}, {402,399}, {399,393}, {395,388}, {391,380}, {389,374}, 
{386,369}, {383,363}, {378,355}, {375,350}, {371,343}, {366,335}, {361,328}, 
{359,325}, {356,319}, {354,314}, {353,314}, {333,474}, {333,466}, {333,460}, 
{333,448}, {333,441}, {333,435}, {333,428}, {333,415}, {333,404}, {334,399}, 
{333,390}, {336,380}, {335,377}, {334,374}, {334,363}, {334,352}, {333,338}, 
{333,333}, {333,328}, {333,323}, {332,318}, {288,466}, {288,450}, {295,433}, 
{299,421}, {306,396}, {309,384}, {311,377}, {317,356}, {321,335}, {323,329}, 
{324,327}, {376,471}, {365,435}, {359,422}, {358,407}, {356,399}, {355,388}, 
{352,378}, {347,363}, {340,347}, {341,333}, {343,324}, {338,311}, {317,309}, 
{317,296}, {322,290}, {336,286}, {342,287}, {346,299}, {352,305}, 
    };


// ini adalah configurasi RBG sensor pada kamera realsense, deteksi masih manual, juga input ke program manual

    //default one config
    //rgbSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1.0f);
    // rgbSensor.set_option(RS2_OPTION_EXPOSURE,75.0f);
    // rgbSensor.set_option(RS2_OPTION_GAIN,64.0f);
    // rgbSensor.set_option(RS2_OPTION_BRIGHTNESS,0.0f);
    // rgbSensor.set_option(RS2_OPTION_CONTRAST,50.0f);
    // rgbSensor.set_option(RS2_OPTION_GAMMA,300.0f);
    // rgbSensor.set_option(RS2_OPTION_HUE,0.0);
    // rgbSensor.set_option(RS2_OPTION_SATURATION,64.0f);
    // rgbSensor.set_option(RS2_OPTION_SHARPNESS,50.0f);
    // rgbSensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,1.f);

    //custom one config
    rgbSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1.0f);
    //rgbSensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE,1.0);
    //rgbSensor.set_option(RS2_OPTION_EXPOSURE,100.0f);
    //rgbSensor.set_option(RS2_OPTION_GAIN,64.0f);
    //rgbSensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY,1.0f);
    rgbSensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION,1.0f);
    rgbSensor.set_option(RS2_OPTION_BRIGHTNESS,0.0f);
    rgbSensor.set_option(RS2_OPTION_CONTRAST,50.0f);
    rgbSensor.set_option(RS2_OPTION_GAMMA,300.0f);
    rgbSensor.set_option(RS2_OPTION_HUE,0.0);
    rgbSensor.set_option(RS2_OPTION_SATURATION,50.0f);
    rgbSensor.set_option(RS2_OPTION_SHARPNESS,100.0f);
    rgbSensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,1.f);

    namedWindow(window_name,WINDOW_AUTOSIZE);

    //libRealSense decimation filter init
    //rs2::decimation_filter dec_filter;
    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,2);

    //image process line detector pointer
    //Ptr<FastLineDetector> fld = createFastLineDetector(3,1.414213538f,318,204,3,true);
    createTrackbar("rho", window_name, &rhoVal, 360, on_rhoChange);    
    createTrackbar("theta", window_name, &thetaVal, 180, on_thetaChange);
    createTrackbar("threshold", window_name, &threshVal, 500, on_threshChange);    
    createTrackbar("minLineLength", window_name, &minLineLen, 500, on_lineLenChange);
    createTrackbar("maxLineGap", window_name, &maxLineGap, 500, on_lineGap);
    createTrackbar("ISA", window_name, &ISA, 1, on_aktivasi);
    //createTrackbar("len_threshold", window_name, &lenThresh, 100, on_lenThresh);
    // createTrackbar("canny_th1", window_name, &cannyTh1, 500, on_cannyTh1);
    // createTrackbar("canny_th2", window_name, &cannyTh2, 500, on_cannyTh2);
    // createTrackbar("do_merge", window_name, &do_merge, 1, on_doMerge);
    create_thread(myThread,myProgram,NULL);
 //   create_thread(myThread2,myProgram2,NULL);
    create_thread(myThread3,myProgram3,NULL);
    create_thread(myThread4,myProgram4,NULL);

    cvKey = cv::waitKey(1);
    while(cvKey!=(int)'q' && getWindowProperty(window_name,WND_PROP_AUTOSIZE)>=0)
    {
        if (cvKey == (int)'s') {
            sButtonPress = true;
        }else {
            sButtonPress = false;
        }
        //move if button press
        if (sButtonPress) {
            speed = speedq;
        } else {
            speed = 0;
        }

        switch (cvKey)
        {
        case (int)'w':
            i2cstartFlag=true;
            mobiljalan=true;
            break;
        
        case (int)'e':
            i2cstartFlag=false;
            mobiljalan=false;
            break;

        case (int)'u':
            inputtele++;
            break;

        case (int)'j':
            inputtele--;
            break;

        case (int)'h':
            steeringtele++;
            break;

        case (int)'k':
            steeringtele--;             
            break;

        case (int)'z':
            braketele++;
            break;

        case (int)'x':
            braketele--;
            break;
        

        default:
            break;
        }
        //block program until frames arrive
        rs2::frameset frames = pipe.wait_for_frames(); 

        //try to get a video
        rs2::video_frame videoFrame= frames.get_color_frame(); // pemulaian pengambilan gambar

        //rs2::video_frame filtered = videoFrame;

        //filtered = dec_filter.process(filtered);

        //query frame size 
        auto w = videoFrame.as<rs2::video_frame>().get_width();
        auto h = videoFrame.as<rs2::video_frame>().get_height();
        
        Mat img(Size(w,h),CV_8UC3,(void*)videoFrame.get_data(), Mat::AUTO_STEP); //main image input 
        //Mat img(Size(w,h),CV_8UC3);
        //rotate(imgx,img,ROTATE_180);


        // disini bisa ditambah proses CNN untuk akurasi warna, untuk saat ini masih  HSV



        Mat img2 = img.clone();
        Mat img3 = img.clone();
        imgUniversal = img.clone();
        Mat imgHSV(Size(w,h),CV_8UC3);
        Mat imgInRangeHSV(Size(w,h),CV_8UC1);
        Mat imgBlur(Size(w,h),CV_8UC1);
        
        // processing 1 bgr to hsv color
        cvtColor(img,imgHSV,COLOR_BGR2HSV);
        //orange isolation
        //inRange(imgHSV,Scalar(9,39,108),Scalar(35,255,255),imgInRangeHSV);
        //green isolation
        //inRange(imgHSV,Scalar(37,45,0),Scalar(86,255,255),imgInRangeHSV);
        inRange(imgHSV,Scalar(41,42,0),Scalar(120,255,255),imgInRangeHSV); //pengaturan warna HSV, bisa CNN untuk mempelajari garisnya, train data yang banyak

        
        GaussianBlur(imgInRangeHSV,imgBlur,Size(5,5),0);

        /* CREATE NEW ROI POLYGON SELECTION */
        Mat mask = Mat(Size(w,h), CV_8UC1,Scalar(0,0,0));
        Mat maskImg = Mat(Size(w,h),CV_8UC1,Scalar(0,0,0));
        std::vector<cv::Point> pts; // untuk seleksi ROI (region of interest), dari deteksi garis yang bersih, hanya dipakai setengah dari bawah
        pts.push_back(Point(27,(int)(h/2)+13));
        pts.push_back(Point((int)(w-27),(int)(h/2)+13));
        pts.push_back(Point((int)(w-0),(int)h));
        pts.push_back(Point(0,(int)h));
        fillConvexPoly(mask,pts,Scalar(255,255,255),LINE_8,0); // gambar trapesium deteksi line , warna putih, untuk membersihkan seleksi deteksi.
        bitwise_and(imgBlur,mask,maskImg);

        //using hough line probabilistics to get best result
        std::vector<cv::Vec4i> lines;
        HoughLinesP(maskImg,lines,rhoVal,thetaVal*CV_PI/180,threshVal,minLineLen,maxLineGap); // fungsi deteksi garis tepi
        //HoughLinesP(maskImg3,lines,rhoVal,thetaVal*CV_PI/180,threshVal,minLineLen,maxLineGap); //1,CV_PI/180,50,50,10
        for(size_t i=0;i<lines.size();i++)
        {
            Vec4i l = lines[i];
            cv::line(img2,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(255,0,0),3,LINE_8);
        }
       

        /* LINE SELECTION */ // ini adalah seleksi garis kanan dan kiri, dan pengelompokannya
        xleft.clear();yleft.clear();xright.clear();yright.clear(); // tidak usah diubah sampai line 833
        float xminleft=0.f,xmaxleft=0.f,xminright=0.f,xmaxright=0.f;
        float yminleft=0.f,ymaxleft=0.f,yminright=0.f,ymaxright=0.f;
        bool leftFirst=true,rightFirst=true;
        bool yleftFirst=true,yrightFirst=true;
        //cout <<"------------------------------" << endl;
        for (int i=0;i<lines.size();i++)
        {
            x.clear();y.clear();out.clear();
            for(int j=0;j<2;j++)
            {
                x.push_back(lines[i][j*2]);
                y.push_back(lines[i][(j*2)+1]);
            }
            
            if(x[0]<middlePoint && x[1]<middlePoint)
            {
                xleft.push_back((float)lines[i][0]);
                xleft.push_back((float)lines[i][2]);
                yleft.push_back((float)CAMERA_HEIGHT-(float)lines[i][1]);
                yleft.push_back((float)CAMERA_HEIGHT-(float)lines[i][3]);
                if(leftFirst==true)
                {
                    xminleft=lines[i][0];
                    xmaxleft=lines[i][0];
                    leftFirst=false;
                    if(lines[i][2]<xminleft)
                    {
						xminleft=lines[i][2];
					}
					else if(lines[i][2]>xmaxleft)
					{
						xmaxleft=lines[i][2];
					}
                }
                else
                {
                    if(lines[i][0]<lines[i][2])
                    {
                        if(lines[i][0]<xminleft)
                        {
                            xminleft=lines[i][0];
                        }
                        if(lines[i][2]>xmaxleft)
                        {
                            xmaxleft=lines[i][2];
                        }
                    }
                    else if(lines[i][0]>lines[i][2])
                    { 
                        if(lines[i][0]>xmaxleft)
                        {
                            xmaxleft=lines[i][0];
                        }
                        if(lines[i][2]<xminleft)
                        {
                            xminleft=lines[i][2];
                        }
                    }
                }
                
                if(yleftFirst==true)
                {
					yminleft=lines[i][1];
					ymaxleft=lines[i][1];
					yleftFirst=false;
					if(lines[i][3]<yminleft)
                    {
						yminleft=lines[i][3];
					}
					else if(lines[i][3]>ymaxleft)
					{
						ymaxleft=lines[i][3];
					}
				}
				else
                {
                    if(lines[i][1]<lines[i][3])
                    {
                        if(lines[i][1]<yminleft)
                        {
                            yminleft=lines[i][1];
                        }
                        if(lines[i][3]>ymaxleft)
                        {
                            ymaxleft=lines[i][3];
                        }
                    }
                    else if(lines[i][1]>lines[i][3])
                    { 
                        if(lines[i][1]>ymaxleft)
                        {
                            ymaxleft=lines[i][1];
                        }
                        if(lines[i][3]<yminleft)
                        {
                            yminleft=lines[i][3];
                        }
                    }
                }
                //cout << lines[i][0] << "|" << lines[i][2] << endl;
                //cout << xleft[xleft.size()-2] << "|" << xleft[xleft.size()-1]<<endl;
                //cout << xleft.size() << endl;
            }
            else if(x[0]>middlePoint && x[1]>middlePoint)
            {
                xright.push_back((float)lines[i][0]);
                xright.push_back((float)lines[i][2]);
                yright.push_back((float)CAMERA_HEIGHT-(float)lines[i][1]);
                yright.push_back((float)CAMERA_HEIGHT-(float)lines[i][3]);
				if(rightFirst==true)
                {
                    xminright=lines[i][0];
                    xmaxright=lines[i][0];
                    rightFirst=false;
                    if(lines[i][2]<xminright)
                    {
						xminright=lines[i][2];
					}
					else if(lines[i][2]>xmaxright)
					{
						xmaxright=lines[i][2];
					}
                }
                else
                {
                    if(lines[i][0]<lines[i][2])
                    {
                        if(lines[i][0]<xminright)
                        {
                            xminright=lines[i][0];
                        }
                        if(lines[i][2]>xmaxright)
                        {
                            xmaxright=lines[i][2];
                        }
                    }
                    else if(lines[i][0]>lines[i][2])
                    { 
                        if(lines[i][0]>xmaxright)
                        {
                            xmaxright=lines[i][0];
                        }
                        if(lines[i][2]<xminright)
                        {
                            xminright=lines[i][2];
                        }
                    }
                }
                
                if(yrightFirst==true)
                {
					yminright=lines[i][1];
					ymaxright=lines[i][1];
					yrightFirst=false;
					if(lines[i][3]<yminright)
                    {
						yminright=lines[i][3];
					}
					else if(lines[i][3]>ymaxright)
					{
						ymaxright=lines[i][3];
					}
				}
				else
                {
                    if(lines[i][1]<lines[i][3])
                    {
                        if(lines[i][1]<yminright)
                        {
                            yminright=lines[i][1];
                        }
                        if(lines[i][3]>ymaxright)
                        {
                            ymaxright=lines[i][3];
                        }
                    }
                    else if(lines[i][1]>lines[i][3])
                    { 
                        if(lines[i][1]>ymaxright)
                        {
                            ymaxright=lines[i][1];
                        }
                        if(lines[i][3]<yminright)
                        {
                            yminright=lines[i][3];
                        }
                    }
                }

            }
        }
        //float ymax1;
        //if(ymaxleft==0)
        //{
			//ymax1=ymaxright;
		//}
		//else if(ymaxright==0)
		//{
			//ymax1=ymaxleft;
		//}
		//else
		//{
			//ymax1=((CAMERA_HEIGHT-ymaxleft)<(CAMERA_HEIGHT-ymaxright))?(ymaxleft):(ymaxright);
		//}
		
        //cout << ymaxleft << "|"<< ymaxright << "," << yminleft << "|" << yminright << endl;
        //cout <<xminleft<<"|"<<xmaxleft<<"|"<<xminright<<"|"<<xmaxright<< endl;
        //cout <<"------------------------------" << endl;
        std::vector<float> coeff;
        std::vector<float> left_fit,right_fit;
        std::array<std::array<float,4>,2> lane_lines;
        int j=0;

        coeff.clear();
        //cout << "--------------------" <<endl;
        //cout << xleft.size() << "|" << yleft.size() << endl;
        // for(int i=0;i<xleft.size();i++)
        // {
        //     cout << xleft[i] << endl;
        // }
        //cout << "--------------------" <<endl;
        coeff = polyfit(xleft,yleft,2); // _+________________________________________garis sudah didapatkan, masuk ke polyfit, ini yang kiri
        //cout << "------------------------------"<<endl;
        //cout << cv::format(xleft,cv::Formatter::FMT_PYTHON) <<"|" <<cv::format(yleft,cv::Formatter::FMT_PYTHON)<< "|"<<cv::format(coeff,cv::Formatter::FMT_PYTHON)<<endl;
        //cout << "------------------------------"<<endl;
        if(coeff[0]!=0 && coeff.size()!=1)
        {
            left_fit=make_points(coeff,false,xminleft,xmaxleft,yminleft);
            for(int i=0;i<4;i++)
            {
                lane_lines[j][i]=left_fit[i];
            }
            j++;
        }
        
        
        coeff.clear();
        //cout << "--------------------" <<endl;
        coeff = polyfit(xright,yright,2); // _________________________________________ini garis kanan
        //cout << "------------------------------"<<endl;
        //cout << cv::format(xright,cv::Formatter::FMT_PYTHON) <<"|" <<cv::format(yright,cv::Formatter::FMT_PYTHON)<< "|"<<cv::format(coeff,cv::Formatter::FMT_PYTHON)<<endl;
        //cout << "------------------------------"<<endl;
        if(coeff[0]!=0 && coeff.size()!=1)
        {
            right_fit=make_points(coeff,true,xminright,xmaxright,yminright);
            for(int i=0;i<4;i++)
            {
                lane_lines[j][i]=right_fit[i];
            }
            j++;
        }

        //if(lane_lines.size()>0)
        //{
            //line(img3,Point((int)lane_lines[0][0],(int)CAMERA_HEIGHT-(int)lane_lines[0][1]),Point((int)lane_lines[0][2],(int)CAMERA_HEIGHT-(int)lane_lines[0][3]),Scalar(255,0,0),2,LINE_8,0);
            //line(img3,Point((int)lane_lines[1][0],(int)CAMERA_HEIGHT-(int)lane_lines[1][1]),Point((int)lane_lines[1][2],(int)CAMERA_HEIGHT-(int)lane_lines[1][3]),Scalar(255,0,0),2,LINE_8,0);
        //}

        auto output = getHeadingAndOffset(lane_lines,j); // pengambilan sudut heading kendaraan
        // cout << "heading angle ="<<output[0] << endl;
        int x1=CAMERA_WIDTH/2;
        int y1=CAMERA_HEIGHT;
        int x2=(CAMERA_WIDTH/2)+output[1];
        int y2=CAMERA_HEIGHT/2;
        line(img3,Point(x1,y1),Point(x2,y2),Scalar(255,0,0),2,LINE_8,0); // menggambari garis tengah dari kamera
        heading_angle = output[0];
        i2cSendData=true; // eksekusi pengiriman I2C ke STM32
        //update window with new data
        imshow(window_name,img3);
        

        // Get depth frame
        rs2::depth_frame depth = frames.get_depth_frame();

        // Vector to store valid distances
        std::vector<float> valid_distances;

        // Get the width and height of the depth frame
        auto width = depth.get_width();
        auto height = depth.get_height();

 
        // Iterate through the coordinates and get distances
        for (const auto& coord : coordinates) {
            int x = coord.first;
            int y = coord.second;
            float distance = depth.get_distance(x, y) * 100;

            if (distance > 125 + (479 - y) * 0.5) {
                distance = 600;
            }
            //  std::cout << "y is: " << y << "p" << std::endl;
            //  std::cout << " distance is: " << distance << " cm" << std::endl;
            
            if (distance > min_distance_cm) {
                valid_distances.push_back(distance);
            }
  

        }

        // Remove distances less than min_distance_cm
        valid_distances.erase(std::remove_if(valid_distances.begin(), valid_distances.end(),
                                             [min_distance_cm](float distance) {
                                                 return distance < min_distance_cm;
                                             }), valid_distances.end());

        // Print the remaining valid distances
        // std::cout << "Valid distances: ";
        // for (const auto &dist : valid_distances) {
        //     std::cout << dist << " cm ";
        // }
        // std::cout << std::endl;

        // Find and display the minimum valid distance
        if (!valid_distances.empty()) {
             jarak_depth = *std::min_element(valid_distances.begin(), valid_distances.end());
            //std::cout << "The closest valid distance is: " << min_distance << " cm" << std::endl;
        } else {
            //std::cout << "jarak aman" << std::endl;
        }


        cvKey = cv::waitKey(1);
    }
	i2cstartFlag=false;
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

extern "C" void* myProgram(void* p) // thread program 1, isinya  mengirim i2c
{
    if((i2cFD=open(devID,O_RDWR)) < 0)
    {
        std::cout << "failed to open i2c bus" << endl;
        return 0;
    }
    else{
        std::cout << "i2c bus open on "<< devID << endl;
    }
    if(ioctl(i2cFD,I2C_SLAVE,devADDR)<0)
    {
        std::cout << "failed to acquire bus access" << endl;
        return 0;
    }
    else{
        std::cout << "i2c bus acquired" << endl;
    }

    ms = get_millis();
    ms4 = get_millis();
    for(;;)
    {
        if(get_millis()-ms>=500) // refresh rate kirim I2C
        {
            // cout << "interruption 500ms " << endl;
            // ms=get_millis();
            if(i2cstartFlag!=i2cLatestStartFlag) 
            {
                if(i2cstartFlag==true)
                {
                    i2c_setBuffer("start");
                    i2c_write(buffer,7);
                    i2cLatestStartFlag=true;

                }
                else if(i2cstartFlag==false)
                {
                    speed=0.25f;
                    //steering=20.0f;    // 0 degree with offset 20.0f
                    steering=0;
                    i2c_setBuffer("set");
                    i2c_write(buffer,7);
                    i2cSendData=false;
                    i2c_setBuffer("stop");
                    i2c_write(buffer,7);
                    i2cLatestStartFlag=false; 
                }
            }
            if(i2cSendData)
            {
                if (inputtele == 10) {
                    speedq = 2.78f;
                } else if (inputtele == 9 ) {
                    speedq = 2.5f;
                } else if (inputtele == 8 ) {
                    speedq = 2.175f;
                } else if (inputtele == 7 ) {
                    speedq = 1.925f;
                } else if (inputtele == 6 ) {
                    speedq = 1.65f;
                } else if (inputtele == 5 ) {
                    speedq = 1.25f;
                } else if (inputtele == 4 ) {
                    speedq = 1.0f;
                } else if (inputtele == 3 ) { 
                    speedq = 0.75f;
                } else if (inputtele == 2 ) { 
                    speedq = 0.50f;
                } else if (inputtele == 1 ) { 
                    speedq = 0.25f;
                } else {
                    speedq = 0; // Kecepatan Tidak Sesuai
                }   

                // if (braketele == 10) {
                //     brake = 2.50f;
                // } else if (braketele == 9) {
                //     brake = 2.25f;
                // } else if (braketele == 8) {
                //     brake = 2.0f;
                // } else if (braketele == 7) {
                //     brake = 1.75f;
                // } else if (braketele == 6) {
                //     brake = 1.50f;
                // } else if (braketele == 5) {
                //     brake = 1.25f;
                // } else if (braketele == 4) {
                //     brake = 1.0f;
                // } else if (braketele == 3) { 
                //     brake = 0.75f;
                // } else if (braketele == 2) { 
                //     brake = 0.50f;
                // } else if (braketele == 1) { 
                //     brake = 0.25f;
                // } else {
                //     brake = 0; // Kecepatan Tidak Sesuai
                //} 
                brake =  braketele;
                cout <<"GT = "<< GT << endl;
                cout <<"ISA = "<< ISA << endl;
                //printf(GT);
                i2c_setBuffer("set");
                i2c_write(buffer,7);
                i2cSendData=false;
            }
        }
        
        if(get_millis()-ms4>=100) // refresh rate untuk kalkulasi line keeping purePursuit
        {
			
			float getSteeringAngle = purePursuitControl(heading_angle);
			cout << "steering angle = "<<getSteeringAngle << endl;
            cout << "steering angle = "<<steeringtele << endl;
            cout << "speedq = "<<speedq << endl;
            cout << "brake = "<<brake << endl;            
			//steering = getSteeringAngle+20.0f; //+offset on motor controller
            //steering = getSteeringAngle;
            steering = steeringtele; 
            
			ms4=get_millis();
		}
    }
}

// extern "C" void* myProgram2(void* p) 
// {
//     struct ftdi_context ctx;
//     if (ftdi_init(&ctx) < 0) {
//         fprintf(stderr, "Failed to initialize FTDI context\n");
//         return 0;
//     }

//     if (ftdi_usb_open(&ctx, 0x0403, 0x6001) < 0) {
//         fprintf(stderr, "Failed to open FTDI device: %s\n", ftdi_get_error_string(&ctx));
//         ftdi_deinit(&ctx);
//         return 0;
//     }

//     if (ftdi_set_baudrate(&ctx, 19200) < 0) {
//         fprintf(stderr, "Failed to set baud rate: %s\n", ftdi_get_error_string(&ctx));
//         ftdi_usb_close(&ctx);
//         ftdi_deinit(&ctx);
//         return 0;
//     }

//     // Receive data continuously
//     unsigned char data_buffer[50]; // Use a buffer to store received data
//     while (1) {
//         int bytes_read = ftdi_read_data(&ctx, data_buffer, sizeof(data_buffer) - 1);
//         if (bytes_read < 0) {
//             fprintf(stderr, "Failed to read data: %s\n", ftdi_get_error_string(&ctx));
//         } else if (bytes_read == 0) {
//             fprintf(stderr, "No data received\n");
//         } else {
//             data_buffer[bytes_read] = '\0'; // Null-terminate the received data

//             // Parse the received data and extract c, adjspeed, distance, and rpmMotorTacho values
//             int num_items = sscanf(reinterpret_cast<const char*>(data_buffer), "%d,%d,%d,%dn", &clsobjdet, &adjspeed, &lidardistance, &rpmMotorTacho);
//             if (num_items == 4) {
//                 // Data successfully parsed
//                 printf("Received data: class = %d, adjspeed = %d, distance = %d, rpmMotorTacho = %d\n", clsobjdet, adjspeed, lidardistance, rpmMotorTacho);
//                 outputRPM = rpmMotorTacho / gearRatio;
//                 speed_kmph = (outputRPM * wheelCircumference * 60.0) / 1000.0;
//             } else {
//                 // Invalid data format
//                 fprintf(stderr, "Invalid data format: %s\n", data_buffer);
//             }
//         }

//         // Add a delay of 0.1 second before reading the next data
//         usleep(100); // 100,000 microseconds = 100 milliseconds = 0.1 seconds
//     }

//     ftdi_usb_close(&ctx);
//     ftdi_deinit(&ctx);

//     return 0;
// }
//ACC
extern "C" void* myProgram3(void* p) // untuk logging kecepatan sudut yang perintah, output .csv, di stm ada sendiri pake laptop, susah wkwkwkwk -Ifan 2022. sehingga kami (penerus 2023) menggunakan 2 buah encoder (baru dan lama) dan yang lama menjadi input di arduino due dan dikirim (datanya rpmMotorTacho dan speed_kmph) 
{	
	while(mobiljalan==false);-0;/////////////////////////
	if(mobiljalan==true)
	{
    try {
       //csvfile csv("Riset2023ISA_logging_02-10-23_0.1ms_60801007090kmphvid.csv");
        csvfile csv("k50variasi1.csv");
        // csv << "ms" << "speed" << "steering" << "clsobjdet" << "GT" << "ISA" << "adjspeed" << "lidardistance" << "rpmMotorTacho" << "speed_kmph" << "InputSpeed" << "InputDistance" << "deptcam_distance" << "brake" << endrow;
        csv << "ms" << "speed" <<  "deptcam_distance" << "brake" << endrow;

        //csv << "ms" << "speed" << "steering" << "clsobjdet" << "GT" << "ISA" << "adjspeed" << "lidardistance" << "rpmMotorTacho" << "deptcam_distance" << endrow;
        ms3=get_millis();
        float c=0.f;
        char myBuffer[8];
        while(true)
        {
            if(get_millis()-ms3>=100)
            {
                ms3=get_millis();
                c=c+0.1f;
                sprintf(myBuffer,"%.1f",c);
                csv << myBuffer << speed << jarak_depth << brake_signal << endrow;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    }
}

extern "C" void* myProgram4(void* p) // untuk logging kecepatan sudut yang perintah, output .csv, di stm ada sendiri pake laptop, susah wkwkwkwk -Ifan 2022. sehingga kami (penerus 2023) menggunakan 2 buah encoder (baru dan lama) dan yang lama menjadi input di arduino due dan dikirim (datanya rpmMotorTacho dan speed_kmph) 
{	
	while(true){/////////////////////////
    std::cout << "jarak_depth: " << jarak_depth << std::endl;
	

       // Define membership functions for inputs
    TriangularMembershipFunction lowspeed(0, 0, 30);
    TriangularMembershipFunction mediumspeed(25, 30, 60);
    TriangularMembershipFunction highspeed1(60, 100, 120);

    // for distanse
    TrapezoidalMembershipFunction lowdistance(0, 0, 40, 40);
    TrapezoidalMembershipFunction mediumdistance(40, 40, 75, 100);
    TrapezoidalMembershipFunction highdistance(75, 200, 100, 200);

    // Create fuzzy controller
    FuzzyController controller;

    // Add rules
    controller.addRule(FuzzyRule(lowspeed, lowdistance, 0));
    controller.addRule(FuzzyRule(lowspeed, mediumdistance, 0));
    controller.addRule(FuzzyRule(lowspeed, highdistance, 0));
    controller.addRule(FuzzyRule(mediumspeed, lowdistance, 50));
    controller.addRule(FuzzyRule(mediumspeed, mediumdistance, 60));
    controller.addRule(FuzzyRule(mediumspeed, highdistance, 70));
    controller.addRule(FuzzyRule(highspeed1, lowdistance, 100));
    controller.addRule(FuzzyRule(highspeed1, mediumdistance, 100));
    controller.addRule(FuzzyRule(highspeed1, highdistance, 80));

    // Evaluate the controller
    double input1 = 30; // Example input 1
    double input2 = jarak_depth; // Example input 2
 


    double output = controller.evaluate(input1, input2);
    brake_signal = output *18;
    std::cout << "Fuzzy Output: " << brake_signal << std::endl;
    }
    return 0;

    
    
}