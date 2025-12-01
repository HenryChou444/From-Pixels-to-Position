#include "lib/rapidcsv.h"
#include "lib/vqf/vqf.hpp"

using namespace std;

#define DATA_PATH       "../Data/random_test/18cm.csv" // move around and get back to around the same starting point
#define COLUMN_1        "Timestamp"
#define COLUMN_2        " Data_type"
#define COLUMN_3        " Gyro x"
#define COLUMN_4        " Gyro y"
#define COLUMN_5        " Gyro z"
#define COLUMN_6        " Accel x"
#define COLUMN_7        " Accel y"
#define COLUMN_8        " Accel z"
#define COLUMN_9        " Filter yaw"
#define COLUMN_10       " Filter pitch"
#define COLUMN_11       " Filter roll"
#define COLUMN_12       " Image_name "

#define NUMBER_OF_GYRO_CAL_DATA     1450

#define GYRO_SAMPLE_FREQ                208
#define GYRO_OFFSET_INIT                0
#define GYRO_BIN_TO_DEG_PR_SEC          0.004375
#define GYRO_DEG_PR_SEC_TO_DEG_PR_SEC   1
#define GYRO_TIMESTAMP_TO_SEC           1000000
#define GYRO_DATA_TYPE_CONVERSION       GYRO_BIN_TO_DEG_PR_SEC //alternative if data are allready in degree: GYRO_DEG_PR_SEC_TODEG_PR_SEC

#define ACCEL_X_OFFSET  0.003111
#define ACCEL_Y_OFFSET  -0.007137
#define ACCEL_Z_OFFSET  -0.002196
#define ACCEL_X_SCALE   -0.994971
#define ACCEL_Y_SCALE   -0.996191
#define ACCEL_Z_SCALE   -1.00016
#define ACCEL_TO_G      0.00006103

#define IMU_SAMPLE_FREQ 208
#define IMU_SAMPLE_TIME 1.0f/(float)(IMU_SAMPLE_FREQ)

#define COMPLIMENTARY_FILTER_IMAGE_WEIGHT   0.02
#define COMPLIMENTARY_FILTER_GYRO_WEIGHT    0.98
#define COMPLIMENTARY_FILTER_ANGLE_INIT     0

#define PI          3.141592653
#define DEG2RAD     PI/180

/*--------- ENUM TYPE ----------*/
enum data_type_t
{
    IMG_DATA = 0,
    IMU_DATA,
    UNKNOWN,
    NONE,
};

struct angle_t
{
    double roll;
    double pitch;
    double yaw;
};

struct raw_gyro_t
{
    double roll;
    double pitch;
    double yaw;
};

struct gyro_t
{
    int sample_freq;
    raw_gyro_t offset;
};

struct pos_t
{
    double x;
    double y;
    double z;
};

struct complimentary_filter_angle_t
{
    angle_t     complimentary_angle;
    angle_t     gyro_angle;
    float       gyro_weight;
    angle_t     image_angle;
    float       image_weight;
    pos_t       accel_position;
    pos_t       accel_velocity;
    pos_t       image_position;
};


/*-------- functions ---------*/
void init_complimentary_filter(complimentary_filter_angle_t &filter)
{
    filter.image_weight                 = COMPLIMENTARY_FILTER_IMAGE_WEIGHT;
    filter.gyro_weight                  = COMPLIMENTARY_FILTER_GYRO_WEIGHT;

    filter.complimentary_angle.pitch    = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.complimentary_angle.roll     = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.complimentary_angle.yaw      = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.gyro_angle.pitch             = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.gyro_angle.roll              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.gyro_angle.yaw               = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_angle.pitch             = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_angle.roll              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_angle.yaw               = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_position.x              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_position.y              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_position.z              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_position.x              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_position.y              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.image_position.z              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_velocity.x              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_velocity.y              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    filter.accel_velocity.z              = COMPLIMENTARY_FILTER_ANGLE_INIT;
    
}
void update_complimentary_filter(complimentary_filter_angle_t *new_data)
{
    new_data->complimentary_angle.pitch = new_data->gyro_angle.pitch * new_data->gyro_weight + new_data->image_angle.pitch * new_data->image_weight;
    new_data->complimentary_angle.roll = new_data->gyro_angle.roll * new_data->gyro_weight + new_data->image_angle.roll * new_data->image_weight;
    new_data->complimentary_angle.yaw = new_data->gyro_angle.yaw * new_data->gyro_weight + new_data->image_angle.yaw * new_data->image_weight;
}
void init_gyro(gyro_t &gyro)
{
    gyro.sample_freq =  GYRO_SAMPLE_FREQ;
    gyro.offset.pitch = GYRO_OFFSET_INIT;
    gyro.offset.roll =  GYRO_OFFSET_INIT;
    gyro.offset.yaw =   GYRO_OFFSET_INIT;
}
void calibrate_gyro(const vector<double> &gyro_data, const vector<string> &data_type, int number_of_data_to_calibrate, double &offset_value)
{
    double temp = 0;
    int imu_count = 0;
    if (number_of_data_to_calibrate > 0)
    {
         for(int i = 0; i < number_of_data_to_calibrate; i++)
        {
            if(data_type[i] == " IMU_DATA")
            {
                temp += gyro_data[i];
                imu_count++;
            } 
        }
        offset_value = temp / imu_count;   
    }
    else
    {
        cout << "number_of_data_to_calibrate is not valid, in gyro calibration" << endl;
    }
}
double gyro_get_angle(double gyro_raw, double old_angle)
{
    double new_angle;
    new_angle = old_angle + (gyro_raw * GYRO_DATA_TYPE_CONVERSION / 208); //(new_time - old_time) / GYRO_TIMESTAMP_TO_SEC);
    return new_angle;
}

double gyro_raw_to_angular_vel(int32_t gyro)
{
    return gyro * GYRO_BIN_TO_DEG_PR_SEC;
}

double gyro_deg_to_rad(double deg)
{
    return deg * DEG2RAD;
}

double raw_accel_to_calibrated_g(int32_t raw_g, double offset, double scale)
{
    double accel_cal = ((raw_g * ACCEL_TO_G) - offset) / scale;
    return accel_cal;
}

double acc_to_pos(double new_g, double old_vel, double old_pos)
{
    double new_vel;
    double new_pos;

    new_vel = old_vel + new_g * IMU_SAMPLE_TIME;
    new_pos = old_pos + new_vel * IMU_SAMPLE_TIME;

    return new_pos;
}


int main()
{
    double start_accel_x;
    double start_accel_y;
    double start_accel_z;

    VQF vqf(IMU_SAMPLE_TIME);
    VQFParams params = vqf.getParams();

    complimentary_filter_angle_t filtered_angle;
    gyro_t imu_gyro;

    init_complimentary_filter(filtered_angle);
    init_gyro(imu_gyro);
    

    rapidcsv::Document doc(DATA_PATH, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(','), rapidcsv::ConverterParams(true));     // Load csv file
    vector<double> timestamp   = doc.GetColumn<double>(COLUMN_1);
    vector<string> data_type   = doc.GetColumn<string>(COLUMN_2);
    vector<double> gyro_x      = doc.GetColumn<double>(COLUMN_3);
    vector<double> gyro_y      = doc.GetColumn<double>(COLUMN_4);
    vector<double> gyro_z      = doc.GetColumn<double>(COLUMN_5);
    vector<int32_t> accel_x    = doc.GetColumn<int32_t>(COLUMN_6);
    vector<int32_t> accel_y    = doc.GetColumn<int32_t>(COLUMN_7);
    vector<int32_t> accel_z    = doc.GetColumn<int32_t>(COLUMN_8);
    // vector<float> filter_yaw   = doc.GetColumn<float>(COLUMN_9);
    // vector<float> filter_pitch = doc.GetColumn<float>(COLUMN_10);
    // vector<float> filter_roll  = doc.GetColumn<float>(COLUMN_11);
    vector<string> image_name  = doc.GetColumn<string>(COLUMN_12);


    calibrate_gyro(gyro_x, data_type, NUMBER_OF_GYRO_CAL_DATA, imu_gyro.offset.pitch);
    calibrate_gyro(gyro_y, data_type, NUMBER_OF_GYRO_CAL_DATA, imu_gyro.offset.roll);
    calibrate_gyro(gyro_z, data_type, NUMBER_OF_GYRO_CAL_DATA, imu_gyro.offset.yaw);

    string type = "";
    int accel_cal_count = 0;
    int temp_count = 0;
    while(temp_count < NUMBER_OF_GYRO_CAL_DATA)
    {
        temp_count++;
        type = data_type[temp_count];
        if(type == " IMU_DATA"){
            start_accel_x += raw_accel_to_calibrated_g(accel_x[accel_cal_count], ACCEL_X_OFFSET, ACCEL_X_SCALE);
            start_accel_y += raw_accel_to_calibrated_g(accel_y[accel_cal_count], ACCEL_Y_OFFSET, ACCEL_Y_SCALE);
            start_accel_z += raw_accel_to_calibrated_g(accel_z[accel_cal_count], ACCEL_Z_OFFSET, ACCEL_Z_SCALE);
            accel_cal_count++;
        }
    }
    start_accel_x = start_accel_x / accel_cal_count;
    start_accel_y = start_accel_y / accel_cal_count;
    start_accel_z = start_accel_z / accel_cal_count;

    cout << "OFFSET:\nRoll:  " << imu_gyro.offset.roll << "\nPitch: " << imu_gyro.offset.pitch << "\nYaw:   " << imu_gyro.offset.yaw << "\nstart X:   " << start_accel_x << "\nstart Y:   " << start_accel_y << "\nstart Z:   " << start_accel_z << endl;

    vqf_real_t acc_incl_g[3] = {0,0,0}; //newest measurement
    vqf_real_t acc_m_pr_ss[3];
    vqf_real_t acc_converted[3] = {0,0,0}; //newest measurement
    vqf_real_t acc_m_pr_ss_incl_g[3];
    vqf_real_t gyro[3] = {0,0,0};
    vqf_real_t start_acc[3] = {start_accel_x*9.81, start_accel_y*9.81, start_accel_z*9.81}; // first measurement
    vqf_real_t quat[4];
    vqf_real_t quatConj[4];
    vqf.update(gyro, start_acc);
    vqf.getQuat6D(quat);
    VQF::quatConj(quat, quatConj);
    VQF::quatRotate(quatConj ,start_acc, start_acc);


    int DATA_TYPE;
    int local_count = 0;
    for(int i = 1; i < timestamp.size(); i++)
    {
        if(data_type[i] == " IMU_DATA")
        {

            gyro[0] = gyro_deg_to_rad(gyro_raw_to_angular_vel(gyro_x[i] - imu_gyro.offset.pitch));
            gyro[1] = gyro_deg_to_rad(gyro_raw_to_angular_vel(gyro_y[i] - imu_gyro.offset.roll));
            gyro[2] = gyro_deg_to_rad(gyro_raw_to_angular_vel(gyro_z[i] - imu_gyro.offset.yaw));

            acc_incl_g[0] = raw_accel_to_calibrated_g(accel_x[i], ACCEL_X_OFFSET, ACCEL_X_SCALE);
            acc_incl_g[1] = raw_accel_to_calibrated_g(accel_y[i], ACCEL_Y_OFFSET, ACCEL_Y_SCALE);
            acc_incl_g[2] = raw_accel_to_calibrated_g(accel_z[i], ACCEL_Z_OFFSET, ACCEL_Z_SCALE);
            acc_m_pr_ss[0] = acc_incl_g[0] * 9.81;
            acc_m_pr_ss[1] = acc_incl_g[1] * 9.81;
            acc_m_pr_ss[2] = acc_incl_g[2] * 9.81;

            vqf.update(gyro, acc_m_pr_ss);
            vqf.getQuat6D(quat);
            VQF::quatConj(quat, quatConj);
            VQF::quatRotate(quatConj , acc_m_pr_ss, acc_m_pr_ss_incl_g);

            //newest converted acc measurement without gravity
            acc_converted[0] = acc_m_pr_ss_incl_g[0] - start_acc[0];
            acc_converted[1] = acc_m_pr_ss_incl_g[1] - start_acc[1];
            acc_converted[2] = acc_m_pr_ss_incl_g[2] - start_acc[2];

            filtered_angle.accel_position.x = acc_to_pos(acc_converted[0], 
                                                       filtered_angle.accel_velocity.x, 
                                                       filtered_angle.accel_position.x);
            filtered_angle.accel_position.y = acc_to_pos(acc_converted[1], 
                                                       filtered_angle.accel_velocity.y, 
                                                       filtered_angle.accel_position.y);
            filtered_angle.accel_position.z = acc_to_pos(acc_converted[2], 
                                                       filtered_angle.accel_velocity.z, 
                                                       filtered_angle.accel_position.z);

            filtered_angle.gyro_angle.pitch = gyro_get_angle((gyro_x[i] - imu_gyro.offset.pitch), filtered_angle.gyro_angle.pitch);
            filtered_angle.gyro_angle.roll = gyro_get_angle((gyro_y[i] - imu_gyro.offset.roll), filtered_angle.gyro_angle.roll);
            filtered_angle.gyro_angle.yaw = gyro_get_angle((gyro_z[i] - imu_gyro.offset.yaw), filtered_angle.gyro_angle.yaw);


        }
        else if(data_type[i] == " IMG_DATA")
        {
            // filtered_angle.image_angle.pitch = 0;
            // filtered_angle.image_angle.roll = 0;
            // filtered_angle.image_angle.yaw = 0;
            // update_complimentary_filter(&filtered_angle);

            //Until the opticalflow has been implemnted here, we just say gyro angle = complimentary angle
            filtered_angle.complimentary_angle.pitch = filtered_angle.gyro_angle.pitch;
            filtered_angle.complimentary_angle.roll = filtered_angle.gyro_angle.roll;
            filtered_angle.complimentary_angle.yaw = filtered_angle.gyro_angle.yaw;
        }
        if(i % (timestamp.size() / 10) == 0)
        {
            local_count++;
            cout << "pos: " <<  sqrt(filtered_angle.accel_position.x * filtered_angle.accel_position.x + filtered_angle.accel_position.y * filtered_angle.accel_position.y) <<
                    ", gyro_data:" <<  filtered_angle.gyro_angle.pitch << 
                    ", " << filtered_angle.gyro_angle.roll <<
                    ", " << filtered_angle.gyro_angle.yaw << endl;
        } 

    }
    
    return 0;
}