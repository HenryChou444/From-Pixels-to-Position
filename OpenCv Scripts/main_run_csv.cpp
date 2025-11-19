#include "lib/rapidcsv.h"

using namespace std;

#define DATA_PATH       "../Data/data.csv" // move around and get back to around the same starting point
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

#define NUMBER_OF_GYRO_CAL_DATA     2500

#define GYRO_SAMPLE_FREQ        208
#define GYRO_OFFSET_INIT        0
#define GYRO_BIN_TO_DEG_PR_SEC  0.004375
#define GYRO_TIMESTAMP_TO_SEC   1000000

#define COMPLIMENTARY_FILTER_IMAGE_WEIGHT   0.02
#define COMPLIMENTARY_FILTER_GYRO_WEIGHT    0.98
#define COMPLIMENTARY_FILTER_ANGLE_INIT     0




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
    int roll;
    int pitch;
    int yaw;
};

struct gyro_t
{
    int sample_freq;
    raw_gyro_t offset;
    

};

struct complimentary_filter_angle_t
{
    angle_t complimentary_angle;
    angle_t  gyro_angle;
    float   gyro_weight;
    angle_t  image_angle;
    float   image_weight;
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
void calibrate_gyro(const vector<int> &gyro_data, const vector<string> &data_type, int number_of_data_to_calibrate, int &offset_value)
{
    int temp = 0;
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
double gyro_get_angle(double gyro_raw, double old_angle, double new_time, double old_time)
{
    double new_angle;
    new_angle = old_angle + (gyro_raw * GYRO_BIN_TO_DEG_PR_SEC * (new_time - old_time) / GYRO_TIMESTAMP_TO_SEC);
    return new_angle;
}


int main()
{
    complimentary_filter_angle_t filtered_angle;
    gyro_t imu_gyro;
    
    rapidcsv::ConverterParams params(true);     // If empty data "0" is inserted
    rapidcsv::Document doc(DATA_PATH, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file

    // Save collumns into vectors
    vector<double> timestamp   = doc.GetColumn<double>(COLUMN_1);
    vector<string> data_type   = doc.GetColumn<string>(COLUMN_2);
    vector<int> gyro_x       = doc.GetColumn<int>(COLUMN_3);
    vector<int> gyro_y       = doc.GetColumn<int>(COLUMN_4);
    vector<int> gyro_z       = doc.GetColumn<int>(COLUMN_5);
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
    cout << "OFFSET:\nRoll:  " << imu_gyro.offset.roll << "\nPitch: " << imu_gyro.offset.pitch << "\nYaw:   " << imu_gyro.offset.yaw << endl;
    cout << "\nDATA:" << endl;
    cout << "-------Timestamp--------|------Roll-----|-----Pitch-----|----Yaw----" << endl;
    
    int DATA_TYPE;
    int local_count = 0;
    for(int i = 1; i < timestamp.size(); i++)
    {
        if(data_type[i] == " IMU_DATA")
        {
            filtered_angle.gyro_angle.pitch = gyro_get_angle(gyro_x[i] - imu_gyro.offset.pitch, filtered_angle.gyro_angle.pitch, timestamp[i], timestamp[i-1]); 
            filtered_angle.gyro_angle.roll = gyro_get_angle(gyro_y[i] - imu_gyro.offset.roll, filtered_angle.gyro_angle.roll, timestamp[i], timestamp[i-1]); 
            filtered_angle.gyro_angle.yaw = gyro_get_angle(gyro_z[i] - imu_gyro.offset.yaw, filtered_angle.gyro_angle.yaw, timestamp[i], timestamp[i-1]); 
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
        if(i % (timestamp.size() / 50) == 0)
        {
            local_count++;
            cout << local_count << ":\t" << timestamp[i]/GYRO_TIMESTAMP_TO_SEC << ",    \t| " << filtered_angle.complimentary_angle.roll << ", \t| " << filtered_angle.complimentary_angle.pitch << ", \t| " << filtered_angle.complimentary_angle.yaw << endl;
        } 

    }
    
    return 0;
}