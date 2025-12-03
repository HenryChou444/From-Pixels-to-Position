#include "lib/rapidcsv.h"

using namespace std;


#define Y_DATA_1       "../Data/Accel_calibration_data/data_No1.csv" 
#define Y_DATA_2       "../Data/Accel_calibration_data/data_No2.csv" 
#define Z_DATA_1       "../Data/Accel_calibration_data/data_No3.csv" 
#define Z_DATA_2       "../Data/Accel_calibration_data/data_No4.csv" 
#define X_DATA_1       "../Data/Accel_calibration_data/data_No5.csv" 
#define X_DATA_2       "../Data/Accel_calibration_data/data_No6.csv" 

#define COLUMN_6        " Accel x"
#define COLUMN_7        " Accel y"
#define COLUMN_8        " Accel z"
#define COLUMN_2        " Data_type"

#define COL_ACCEL_X     COLUMN_6
#define COL_ACCEL_Y     COLUMN_7
#define COL_ACCEL_Z     COLUMN_8
#define COL_DATA_TYPE   COLUMN_2

#define ACCEL_TO_G      0.000061


double get_average(vector<int32_t> data, vector<string> data_type, int data_size)
{
    int temp = 0;
    int counter = 0;

    for(int i = 0; i < data_size; i++)
    {
        if(data_type[i] == " IMU_DATA")
        {
            temp += data[i];
            counter++;
        }
    }

    return (temp / counter);
}


int main()
{
    rapidcsv::ConverterParams params(true);

    rapidcsv::Document doc_x1(X_DATA_1, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file
    rapidcsv::Document doc_x2(X_DATA_2, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file
    rapidcsv::Document doc_y1(Y_DATA_1, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file
    rapidcsv::Document doc_y2(Y_DATA_2, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file
    rapidcsv::Document doc_z1(Z_DATA_1, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file
    rapidcsv::Document doc_z2(Z_DATA_2, rapidcsv::LabelParams(), rapidcsv::SeparatorParams(), params);     // Load csv file

    vector<int32_t> x_1 = doc_x1.GetColumn<int32_t>(COL_ACCEL_X);
    vector<int32_t> x_2 = doc_x2.GetColumn<int32_t>(COL_ACCEL_X);
    vector<int32_t> y_1 = doc_y1.GetColumn<int32_t>(COL_ACCEL_Y);
    vector<int32_t> y_2 = doc_y2.GetColumn<int32_t>(COL_ACCEL_Y);
    vector<int32_t> z_1 = doc_z1.GetColumn<int32_t>(COL_ACCEL_Z);
    vector<int32_t> z_2 = doc_z2.GetColumn<int32_t>(COL_ACCEL_Z);
    vector<string> x_1_data_type = doc_x1.GetColumn<string>(COL_DATA_TYPE);
    vector<string> x_2_data_type = doc_x2.GetColumn<string>(COL_DATA_TYPE);
    vector<string> y_1_data_type = doc_y1.GetColumn<string>(COL_DATA_TYPE);
    vector<string> y_2_data_type = doc_y2.GetColumn<string>(COL_DATA_TYPE);
    vector<string> z_1_data_type = doc_z1.GetColumn<string>(COL_DATA_TYPE);
    vector<string> z_2_data_type = doc_z2.GetColumn<string>(COL_DATA_TYPE);
    

    double x_offset = double(ACCEL_TO_G * (get_average(x_1, x_1_data_type, x_1.size()) + get_average(x_2, x_2_data_type, x_2.size())) / 2);
    double y_offset = double(ACCEL_TO_G * (get_average(y_1, y_1_data_type, y_1.size()) + get_average(y_2, y_2_data_type, y_2.size())) / 2);
    double z_offset = double(ACCEL_TO_G * (get_average(z_1, z_1_data_type, z_1.size()) + get_average(z_2, z_2_data_type, z_2.size())) / 2);
    cout << "X-offset: " << x_offset << "\nY-offset: " << y_offset << "\nZ-offset: " << z_offset << endl;

    double x_scale = double(ACCEL_TO_G * (get_average(x_2, x_2_data_type, x_2.size()) - get_average(x_1, x_1_data_type, x_1.size())) / 2);
    double y_scale = double(ACCEL_TO_G * (get_average(y_2, y_2_data_type, y_2.size()) - get_average(y_1, y_1_data_type, y_1.size())) / 2);
    double z_scale = double(ACCEL_TO_G * (get_average(z_2, z_2_data_type, z_2.size()) - get_average(z_1, z_1_data_type, z_1.size())) / 2);
    cout << "\nX-scale: " << x_scale << "\nY-scale: " << y_scale << "\nZ-scale: " << z_scale << endl;


    return 0;
}