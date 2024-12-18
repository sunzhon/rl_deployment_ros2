// ambotw1 robot node IMU  code
// author: Tao Sun
// Date: 2024-11-19
#include "yesense.hpp"
#define RECV_TIMEOUT_THR    20  /*unit is equal to the above rate*/

namespace yesense{
	YesenseDriver::YesenseDriver(std::string port, int baudrate):port_(port)
				      , baudrate_(baudrate), buffer_size_(4096), configured_(false)
	{

		// reset 
		memset((uint8_t *)&yesense_out, 0, sizeof(yesense_out));
		memset((uint8_t *)&response, 0, sizeof(response));

		// open port
		initSerial();

		// 数据缓冲区
		data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(buffer_size_));

		// 读取串口数据所需的变量
		bytes_    = 0;

		//fetch imu data thread
		//fetch_data_thread_ = boost::thread(boost::bind(&YesenseDriver::fetchImuData,this));
		//fetch_data_thread_.join();

		//process imu data thread
		//process_data_thread_ = boost::thread(boost::bind(&YesenseDriver::processImuData,this));
		//process_data_thread_.join();

	}



	YesenseDriver::~YesenseDriver()
	{
		if(serial_.isOpen())
		{
			serial_.close();
		std::cout<<"Close yesense device"<<std::endl;
		}
		data_buffer_ptr_.reset();

		configured_ = false;
	}


	void YesenseDriver::initSerial()
	{
		try
		{
			serial_.setPort(port_);
			serial_.setBaudrate(baudrate_);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			serial_.setTimeout(to);
			serial_.open();
			std::cout <<  "Sucessfully open imu device:" << port_.c_str() << ", buad rate:"<<baudrate_<<  std::endl;
		}
		catch (serial::IOException &e)
		{
			std::cout <<  "Unable to open imu device:" << port_.c_str() << ", buad rate:"<<baudrate_<<  std::endl;
			std::cout << "exit" <<std::endl;
			exit(-1);
		}


		if (serial_.isOpen())
		{
			std::cout<<"Serial port: " << serial_.getPort().c_str() << " is opened and initialized" <<std::endl;

			configured_ = true;
		}
	}

	void YesenseDriver::fetchImuData()
	{   
		try 
		{
			if(configured_)
			{
				//read data from serial
				if (serial_.available())
				{
					data_ = serial_.read(serial_.available());

					{
						boost::mutex::scoped_lock lock(m_mutex_data_buffer_);  
						for(uint8_t i=0;i<data_.length();i++)
						{
							data_buffer_ptr_->push_back(data_[i]);
						}
					}
					std::cout<<"yesense serial is available"<<std::endl;
				}
				//boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
					std::cout<<"yesense serial is NOT available"<<std::endl;
			}

		} 
		catch (std::exception &err) 
		{
			perror("error in fetchImuData_ function ");
		}
	}

	void YesenseDriver::processImuData(){

		//uint16_t tid = 0x00;
		//uint16_t prev_tid = 0x00;

		uint32_t recv_len = 0;      
		uint32_t timeout_cnt = 0;

		if(configured_)
		{
			// analysze fetch imu data
			recv_len = data_buffer_ptr_->size();
			if(recv_len > 0)
			{          
				boost::mutex::scoped_lock lock(m_mutex_data_buffer_);                  
				for(unsigned int i = 0; i < recv_len; i++)
				{
					message_in_[bytes_ + i] = data_buffer_ptr_->begin()[i];                    
				}
				bytes_ += recv_len;
				data_buffer_ptr_->erase_begin(recv_len);     
			}

			/*analysis data from message_in_ when total recieved data length is bigger than PROTOCOL_MIN_LEN*/
			if(bytes_ >= PROTOCOL_MIN_LEN)
			{
				int cnt = bytes_;
				int pos = 0;
				int ret = 0;
				while(cnt > 0)
				{
					{
					boost::mutex::scoped_lock lock(m_mutex_yesense_out_);                  
					ret = analysis_data(message_in_ + pos, cnt, &yesense_out, &response);
					}

					/*did not find frame header*/
					if(analysis_done == ret)
					{
						pos++;
						cnt--;
					}
					else if(data_len_err == ret)
					{
						if(timeout_cnt >= RECV_TIMEOUT_THR)
						{
							timeout_cnt = 0;
							cnt = 0;
						}

						break;
					}
					else if(crc_err == ret || analysis_ok == ret)
					{

						output_data_header_t *header = (output_data_header_t *)(message_in_ + pos);
						unsigned int frame_len = header->len + PROTOCOL_MIN_LEN;
						if(response.response_recv_done)
						{
							frame_len = response.len + PROTOCOL_MIN_LEN;
						}

						if(analysis_ok == ret)
						{
		  					/*					
							if(response.response_recv_done)
							{
								//ROS_INFO("response recv");
								boost::mutex::scoped_lock lock(m_response_mutex_);
								response.response_need = false;
								response.response_found = false;
								response.response_recv_done = false;
								//proc_response(response, message_in_ + pos + YIS_RESPONSE_CMD_DATA_POS);
							}
							else
							{
								//ROS_INFO("tid is %d", yesense_out.tid);
								if(prev_tid != 0 && tid > prev_tid && prev_tid != tid - 1) 
								{
									printf("Frame losed: prev_TID: %d, cur_TID: %d", prev_tid, tid);
								}

								prev_tid = tid;
								//boost::mutex::scoped_lock lock(m_mutex_); 
								//publish_imu(yesense_out);
							} 

						     */				
						}

						cnt -= frame_len;
						pos += frame_len;                    
					}
				}
				memcpy(message_in_, message_in_ + pos, cnt);
				bytes_ = cnt;                         
			}
			//boost::this_thread::sleep_for(boost::chrono::milliseconds(2));

		}
	}

	protocol_info_t YesenseDriver::readImuData(){
		boost::mutex::scoped_lock lock(m_mutex_yesense_out_); 
		return yesense_out;
	}
}



