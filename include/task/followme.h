#pragma once

#include "task/base.hpp"

class FollowMe : public BasicTask {
public:
    FollowMe() = default;
    ~FollowMe(){
        release();
    }
public:
    void init_task(nlohmann::json task_config = nlohmann::json()) override{

    }
    void reset() override{
        first_frame = true;
    }
    void run(std::shared_ptr<InferenceData_t> infer_data, std::shared_ptr<Engine> engine) override{
        engine->inference(infer_data, "followme");
        if(first_frame && infer_data->output.track_output.bboxes.size() > 0){
            target_id = infer_data->output.track_output.track_ids[0];
            int max_box_area = infer_data->output.track_output.box_areas[0];
            for(size_t i = 1; i < infer_data->output.track_output.bboxes.size(); i++){
                if(infer_data->output.track_output.box_areas[i] > max_box_area){
                    max_box_area = infer_data->output.track_output.box_areas[i];
                    target_id = infer_data->output.track_output.track_ids[i];
                }
            }
            first_frame = false;
        }
        int flag = 0;
        for(int i = 0; i < infer_data->output.track_output.bboxes.size(); i++){
            if(infer_data->output.track_output.track_ids[i] == target_id){
                infer_data->output.track_output.track_names[i] += " (target)";
                flag = 1;

                lost_target_frame_count = 0;
                lost_target = false;
                break;
            }
        }

        if(flag == 0){
            lost_target_frame_count++;
        }

        {
            if(lost_target_frame_count > 30){
                // reset
                first_frame = true;
                lost_target_frame_count = 0;
                lost_target = true;
            }
        }

        infer_data->output.track_output.lost_target = lost_target;


        // judge_lost_target();
    }
    void release() override{}
    // void judge_lost_target(){
    //     if(lost_target_frame_count > 30){
    //         first_frame = true;
    //         lost_target_frame_count = 0;
    //         lost_target = true;
    //     }
    // }
public:
    bool lost_target = true;

    bool first_frame = true;
    int lost_target_frame_count = 0;
    int target_id = -1;
};
