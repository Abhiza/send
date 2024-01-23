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
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


using namespace std;




std::vector<float> x,y,out; // vector buffer untuk sebelum masuk ke vektor bawah ini
std::vector<float> xright,yright,xleft,yleft; // ini vektornya
float slope,intercept; // definisi untuk perhitungan dibawah

const auto window_name="myCamera"; // nama di GUI opencv

/* Data yang akan diterima dari arduino due*/
int clsobjdet, adjspeed;
int GT = 1;
int AEB = 1;// this is to on or of aeb
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

//data jarak gabungan
int distance

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

// tergantung banyak tread yang digunakan diatas
my_Thread myThread; // tergantung banyak tread yang digunakan diatas
my_Thread myThread2;
my_Thread myThread3;
my_Thread myThread4;

//get time on C++ linux
long long int ms,ms2,ms3,ms4;
struct timeval tv;
long int get_millis(void)
{
    gettimeofday(&tv,NULL);
    return ((tv.tv_sec * 1000) + (tv.tv_usec / 1000));
}

int main(int argc, char * argv[]) // main program untuk eksekusi semua fungsi
// int main(int argc, char * argv[])
{
    static bool first = true;
     // createTrackbar("do_merge", window_name, &do_merge, 1, on_doMerge);
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
    create_thread(myThread2,myProgram2,NULL);
    create_thread(myThread3,myProgram3,NULL);
    create_thread(myThread4,myProgram4,NULL);
    i2cSendData=true; // eksekusi pengiriman I2C ke STM32
    return EXIT_SUCCESS;
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
                    i2c_write(buffer,5);
                    i2cLatestStartFlag=true;

                }
                else if(i2cstartFlag==false)
                {
                    speed=0.25f;
                    //steering=20.0f;    // 0 degree with offset 20.0f
                    steering=0;
                    i2c_setBuffer("set");
                    i2c_write(buffer,5);
                    i2cSendData=false;
                    i2c_setBuffer("stop");
                    i2c_write(buffer,5);
                    i2cLatestStartFlag=false; 
                }
            }
            if(i2cSendData)
            {
               if (AEB == 1 && distance > 80 && 0 < speed_kmph <= 30){
                    
                    std::cout << "mode 1 safe zone" << std::endl;
                } else if (AEB == 0 && distance <= 80 && 0 < speed_kmph <= 30) {
                    
                    std::cout << "Mode 1 warning zone" << std::endl;
                } else if (AEB == 1 && distance <= 40 && 0 < speed_kmph <= 30) {
                    speed = 0f;
                    std::cout << "mode 1 critical zone" << std::endl;
                } else if (AEB == 1 && adjspeed <= 60 && distance >= 100 && 30 < speed_kmph <= 60) {
                   
                    std::cout << "mode 2 safe zone" << std::endl;
                } else if (AEB == 1 && distance <= 100 && 30 < speed_kmph <= 60) {
                    
                    std::cout << "mode 2 warning zone" << std::endl;
                } else if (AEB == 1 && distance <= 60 && 30 < speed_kmph <= 60) {
                    speed = 0f;
                    std::cout << "mode 2 critical zone" << std::endl;
                } else if (AEB == 1 && distance >= 120 && 60 < speed_kmph <= 90) {
                  
                    std::cout << "mode 3 safe zone" << std::endl;
                } else if (AEB == 1 && distance <= 120 && 60 < speed_kmph <= 90) {
               
                    std::cout << "mode 3 warning zone" << std::endl;
                } else if (AEB == 1 && distance <= 80 && 60 < speed_kmph <= 90) {
                    speed = 0f;
                    std::cout << "mode 3 critical zone " << std::endl;
                } else if (AEB == 1 && distance >= 140 && 90 < speed_kmph <= 120) {
               
                    std::cout << "mode 4 critical zone" << std::endl;
                } else if (AEB == 1 && distance <= 140 && 90 < speed_kmph <= 120) {
              
                    std::cout << "mode 4 critical zone" << std::endl;
                } else if (AEB == 1 && distance <= 100 && 90 < speed_kmph <= 120) {
                    speed = 0f;
                    std::cout << "mode 4 critical zone" << std::endl;
                } else {
                    speed = 0.0f;
                    std::cout << "kecepatan = 0 km/jam" << std::endl;
                }
                cout <<"AEB = "<< GT << endl;
                //printf(GT);
                i2c_setBuffer("set");
                i2c_write(buffer,5);
                i2cSendData=false;
            }
        }
        
        if(get_millis()-ms4>=100) // refresh rate untuk kalkulasi line keeping purePursuit
        {
			
			float getSteeringAngle = purePursuitControl(heading_angle);
			cout << "steering angle = "<<getSteeringAngle << endl;
			//steering = getSteeringAngle+20.0f; //+offset on motor controller
            steering = getSteeringAngle;
			ms4=get_millis();
		}
    }
}

extern "C" void* myProgram2(void* p) 
{
    struct ftdi_context ctx;
    if (ftdi_init(&ctx) < 0) {
        fprintf(stderr, "Failed to initialize FTDI context\n");
        return 0;
    }

    if (ftdi_usb_open(&ctx, 0x0403, 0x6001) < 0) {
        fprintf(stderr, "Failed to open FTDI device: %s\n", ftdi_get_error_string(&ctx));
        ftdi_deinit(&ctx);
        return 0;
    }

    if (ftdi_set_baudrate(&ctx, 19200) < 0) {
        fprintf(stderr, "Failed to set baud rate: %s\n", ftdi_get_error_string(&ctx));
        ftdi_usb_close(&ctx);
        ftdi_deinit(&ctx);
        return 0;
    }

    // Receive data continuously
    unsigned char data_buffer[50]; // Use a buffer to store received data
    while (1) {
        int bytes_read = ftdi_read_data(&ctx, data_buffer, sizeof(data_buffer) - 1);
        if (bytes_read < 0) {
            fprintf(stderr, "Failed to read data: %s\n", ftdi_get_error_string(&ctx));
        } else if (bytes_read == 0) {
            fprintf(stderr, "No data received\n");
        } else {
            data_buffer[bytes_read] = '\0'; // Null-terminate the received data

            // Parse the received data and extract c, adjspeed, distance, and rpmMotorTacho values
            int num_items = sscanf(reinterpret_cast<const char*>(data_buffer), "%d,%d,%d,%dn", &clsobjdet, &adjspeed, &lidardistance, &rpmMotorTacho);
            if (num_items == 4) {
                // Data successfully parsed
                printf("Received data: class = %d, adjspeed = %d, distance = %d, rpmMotorTacho = %d\n", clsobjdet, adjspeed, lidardistance, rpmMotorTacho);
                outputRPM = rpmMotorTacho / gearRatio;
                speed_kmph = (outputRPM * wheelCircumference * 60.0) / 1000.0;
            } else {
                // Invalid data format
                fprintf(stderr, "Invalid data format: %s\n", data_buffer);
            }
        }

        // Add a delay of 0.1 second before reading the next data
        usleep(100); // 100,000 microseconds = 100 milliseconds = 0.1 seconds
    }

    ftdi_usb_close(&ctx);
    ftdi_deinit(&ctx);

    return 0;
}
//ACC
extern "C" void* myProgram3(void* p) // untuk logging kecepatan sudut yang perintah, output .csv, di stm ada sendiri pake laptop, susah wkwkwkwk -Ifan 2022. sehingga kami (penerus 2023) menggunakan 2 buah encoder (baru dan lama) dan yang lama menjadi input di arduino due dan dikirim (datanya rpmMotorTacho dan speed_kmph) 
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
}

extern "C" void* myProgram4(void* p) // untuk logging kecepatan sudut yang perintah, output .csv, di stm ada sendiri pake laptop, susah wkwkwkwk -Ifan 2022. sehingga kami (penerus 2023) menggunakan 2 buah encoder (baru dan lama) dan yang lama menjadi input di arduino due dan dikirim (datanya rpmMotorTacho dan speed_kmph) 
{	
	while(mobiljalan==false);-0;/////////////////////////
	if(mobiljalan==true)
	{
    try
    {
        csvfile csv("Riset2023_logging_21-09-23_0.5ms_1.csv");
        csv << "ms" << "speed" << "steering" << "clsobjdet" << "adjspeed" << "lidardistance" << "rpmMotorTacho" << "speed_kmph" << endrow;

        ms3=get_millis();
        float c=0.f;
        char myBuffer[8];
        while(true)
        {
            if(get_millis()-ms3>=500)
            {
                ms3=get_millis();
                c=c+0.5f;
                sprintf(myBuffer,"%.1f",c);
                csv << myBuffer << speed << steering << clsobjdet << adjspeed << lidardistance << rpmMotorTacho << speed_kmph << endrow;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
	}
}