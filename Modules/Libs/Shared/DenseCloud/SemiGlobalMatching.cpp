#include "SemiGlobalMatching.h"
#include "sgm_util.h"
#include <chrono>
#include <cassert>
using namespace std::chrono;

SemiGlobalMatching::SemiGlobalMatching() : width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
census_left_(nullptr), census_right_(nullptr),
cost_init_(nullptr), cost_aggr_(nullptr),
cost_aggr_1_(nullptr), cost_aggr_2_(nullptr),
cost_aggr_3_(nullptr), cost_aggr_4_(nullptr),
cost_aggr_5_(nullptr), cost_aggr_6_(nullptr),
cost_aggr_7_(nullptr), cost_aggr_8_(nullptr),
disp_left_(nullptr), disp_right_(nullptr),
is_initialized_(false)
{
}


SemiGlobalMatching::~SemiGlobalMatching()
{
    Release();
    is_initialized_ = false;
}


bool SemiGlobalMatching::Initialize(const sint32& width, const sint32& height, const SGMOption& option)
{
    width_ = width;
    height_ = height;

    option_ = option;

    if (width == 0 || height == 0)
    {
        return false;
    }

    const sint32 img_size = width * height;
    if (option.census_size == Census5x5)
    {
        census_left_ = new uint32[img_size]();
        census_right_ = new uint32[img_size]();
    }
    else
    {
        census_left_ = new uint64[img_size]();
        census_right_ = new uint64[img_size]();
    }

    const sint32 disp_range = option.max_disparity - option.min_disparity;
    if (disp_range <= 0) {
        return false;
    }

    // ƥ����ۣ���ʼ/�ۺϣ�
    const sint32 size = width * height * disp_range;
    cost_init_ = new uint8[size]();
    cost_aggr_ = new uint16[size]();
    cost_aggr_1_ = new uint8[size]();
    cost_aggr_2_ = new uint8[size]();
    cost_aggr_3_ = new uint8[size]();
    cost_aggr_4_ = new uint8[size]();
    cost_aggr_5_ = new uint8[size]();
    cost_aggr_6_ = new uint8[size]();
    cost_aggr_7_ = new uint8[size]();
    cost_aggr_8_ = new uint8[size]();

    // �Ӳ�ͼ
    disp_left_ = new float32[img_size]();
    disp_right_ = new float32[img_size]();

    is_initialized_ = census_left_ && census_right_ && cost_init_ && cost_aggr_ && disp_left_;

    return is_initialized_;
}

bool SemiGlobalMatching::Match(const uint8* img_left, const uint8* img_right, float32* disp_left)
{
    if (!is_initialized_) {
        return false;
    }
    if (img_left == nullptr || img_right == nullptr) {
        return false;
    }

    img_left_ = img_left;
    img_right_ = img_right;

    auto start = std::chrono::steady_clock::now();

    // census�任
    CensusTransform();

    // ���ۼ���
    ComputeCost();

    auto end = steady_clock::now();
    auto tt = duration_cast<milliseconds>(end - start);
    printf("computing cost! timing :	%lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();

    // ���۾ۺ�
    CostAggregation();


    end = steady_clock::now();
    tt = duration_cast<milliseconds>(end - start);
    printf("cost aggregating! timing :	%lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();

    // �Ӳ����
    ComputeDisparity();

    end = steady_clock::now();
    tt = duration_cast<milliseconds>(end - start);
    printf("computing disparities! timing :	%lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();


    // ����Ӳ�ͼ
    memcpy(disp_left, disp_left_, height_ * width_ * sizeof(float32));

    return true;
}

void SemiGlobalMatching::CensusTransform() const
{
    // ����Ӱ��census�任
    if (option_.census_size == Census5x5) {
        sgm_util::census_transform_5x5(img_left_, static_cast<uint32*>(census_left_), width_, height_);
        sgm_util::census_transform_5x5(img_right_, static_cast<uint32*>(census_right_), width_, height_);
    }
    else {
        sgm_util::census_transform_9x7(img_left_, static_cast<uint64*>(census_left_), width_, height_);
        sgm_util::census_transform_9x7(img_right_, static_cast<uint64*>(census_right_), width_, height_);
    }
}

void SemiGlobalMatching::ComputeCost() const
{
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // ������ۣ�����Hamming���룩
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            // ���Ӳ�������ֵ
            for (sint32 d = min_disparity; d < max_disparity; d++) {
                auto& cost = cost_init_[i * width_ * disp_range + j * disp_range + (d - min_disparity)];
                if (j - d < 0 || j - d >= width_) {
                    cost = UINT8_MAX / 2;
                    continue;
                }
                if (option_.census_size == Census5x5) {
                    // ��Ӱ��censusֵ
                    const auto& census_val_l = static_cast<uint32*>(census_left_)[i * width_ + j];
                    // ��Ӱ���Ӧ����censusֵ
                    const auto& census_val_r = static_cast<uint32*>(census_right_)[i * width_ + j - d];
                    // ����ƥ�����
                    cost = sgm_util::Hamming32(census_val_l, census_val_r);
                }
                else {
                    const auto& census_val_l = static_cast<uint64*>(census_left_)[i * width_ + j];
                    const auto& census_val_r = static_cast<uint64*>(census_right_)[i * width_ + j - d];
                    cost = sgm_util::Hamming64(census_val_l, census_val_r);
                }
            }
        }
    }
}

void SemiGlobalMatching::CostAggregation() const
{
    // ·���ۺ�
   // 1����->��/��->��
   // 2����->��/��->��
   // 3������->����/����->����
   // 4������->����/����->����
   //
   // �K �� �L   5  3  7
   // ��    ��	 1    2
   // �J �� �I   8  4  6
   //
    const auto& min_disparity = option_.min_disparity;
    const auto& max_disparity = option_.max_disparity;
    assert(max_disparity > min_disparity);

    const sint32 size = width_ * height_ * (max_disparity - min_disparity);
    if (size <= 0) {
        return;
    }

    const auto& P1 = option_.p1;
    const auto& P2_Int = option_.p2_init;

    if (option_.num_paths == 4 || option_.num_paths == 8) {
        // ���Ҿۺ�
        sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_1_, true);
        sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_2_, false);
        // ���¾ۺ�
        sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_3_, true);
        sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_4_, false);
    }

    if (option_.num_paths == 8) {
        // �Խ���1�ۺ�
        sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_5_, true);
        sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_6_, false);
        // �Խ���2�ۺ�
        sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_7_, true);
        sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_8_, false);
    }

    // ��4/8�����������
    for (sint32 i = 0; i < size; i++) {
        if (option_.num_paths == 4 || option_.num_paths == 8) {
            cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
        }
        if (option_.num_paths == 8) {
            cost_aggr_[i] += cost_aggr_5_[i] + cost_aggr_6_[i] + cost_aggr_7_[i] + cost_aggr_8_[i];
        }
    }
}

void SemiGlobalMatching::ComputeDisparity() const
{
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // ��Ӱ���Ӳ�ͼ
    const auto disparity = disp_left_;
    // ��Ӱ��ۺϴ�������
    const auto cost_ptr = cost_aggr_;

    const sint32 width = width_;
    const sint32 height = height_;
    const bool is_check_unique = option_.is_check_unique;
    const float32 uniqueness_ratio = option_.uniqueness_ratio;

    // Ϊ�˼ӿ��ȡЧ�ʣ��ѵ������ص����д���ֵ�洢���ֲ�������
    std::vector<uint16> cost_local(disp_range);

    // ---�����ؼ��������Ӳ�
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 min_cost = UINT16_MAX;
            uint16 sec_min_cost = UINT16_MAX;
            sint32 best_disparity = 0;

            // ---�����ӲΧ�ڵ����д���ֵ�������С����ֵ����Ӧ���Ӳ�ֵ
            for (sint32 d = min_disparity; d < max_disparity; d++) {
                const sint32 d_idx = d - min_disparity;
                const auto& cost = cost_local[d_idx] = cost_ptr[i * width * disp_range + j * disp_range + d_idx];
                if (min_cost > cost) {
                    min_cost = cost;
                    best_disparity = d;
                }
            }

            if (is_check_unique) {
                // �ٱ���һ�Σ��������С����ֵ
                for (sint32 d = min_disparity; d < max_disparity; d++) {
                    if (d == best_disparity) {
                        // ������С����ֵ
                        continue;
                    }
                    const auto& cost = cost_local[d - min_disparity];
                    sec_min_cost = std::min(sec_min_cost, cost);
                }

                // �ж�Ψһ��Լ��
                // ��(min-sec)/min < min*(1-uniquness)����Ϊ��Ч����
                if (sec_min_cost - min_cost <= static_cast<uint16>(min_cost * (1 - uniqueness_ratio))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }

            // ---���������
            if (best_disparity == min_disparity || best_disparity == max_disparity - 1) {
                disparity[i * width + j] = Invalid_Float;
                continue;
            }
            // �����Ӳ�ǰһ���Ӳ�Ĵ���ֵcost_1����һ���Ӳ�Ĵ���ֵcost_2
            const sint32 idx_1 = best_disparity - 1 - min_disparity;
            const sint32 idx_2 = best_disparity + 1 - min_disparity;
            const uint16 cost_1 = cost_local[idx_1];
            const uint16 cost_2 = cost_local[idx_2];
            // ��һԪ�������߼�ֵ
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * min_cost);
            disparity[i * width + j] = static_cast<float32>(best_disparity) + static_cast<float32>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SemiGlobalMatching::ComputeDisparityRight() const
{
}

void SemiGlobalMatching::LRCheck()
{
}

void SemiGlobalMatching::FillHolesInDispMap()
{
}

void SemiGlobalMatching::Release()
{
    // �ͷ��ڴ�
    SAFE_DELETE(census_left_);
    SAFE_DELETE(census_right_);
    SAFE_DELETE(cost_init_);
    SAFE_DELETE(cost_aggr_);
    SAFE_DELETE(cost_aggr_1_);
    SAFE_DELETE(cost_aggr_2_);
    SAFE_DELETE(cost_aggr_3_);
    SAFE_DELETE(cost_aggr_4_);
    SAFE_DELETE(cost_aggr_5_);
    SAFE_DELETE(cost_aggr_6_);
    SAFE_DELETE(cost_aggr_7_);
    SAFE_DELETE(cost_aggr_8_);
    SAFE_DELETE(disp_left_);
    SAFE_DELETE(disp_right_);
}
