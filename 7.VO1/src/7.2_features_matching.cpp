#include "7.2_features_matching.hpp"

void find_feature_matches(
    cv::Mat img_1, cv::Mat img_2,
    std::vector<KeyPoint> keypoints_1, 
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches_optimized
)
{
    //-- 初始化
    cv::Ptr<ORB> orb = cv::ORB::create(
        500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    
    //-- 第一步：检测 Oriented FAST 角点位置
    orb->detect(img_1, keypoints_1);
    orb->detect(img_2, keypoints_2);
    // 画出检测出来的关键点（只画了img_1）
    cv::Mat show;
    drawKeypoints(
        img_1, keypoints_1, show, 
        cv::Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    cv::imshow("OFAST_points_img1", show);
    cv::waitKey(0);
    //-- 第二步：根据角点位置计算 BRIEF 描述子
    cv::Mat descriptors_1, descriptors_2;
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步：对两幅图像中的 BRIEF 描述子进行匹配，使用 Hamming 距离
    std::vector<DMatch> descriptor_pre_matches;
    cv::BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, descriptor_pre_matches);

    //-- 第四步：筛选匹配点对
    double min_dist = 10000, max_dist = 0;
    // 找出所有匹配之间的最小距离和最大距离
    // 即最相似的和最不相似的亮点之间的距离
    for (int i = 0; i < descriptor_pre_matches.size(); i++)
    {
        double dist = descriptor_pre_matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    printf("Max dist: %f\n", max_dist);
    printf("Min dist: %f\n", min_dist);
    
    // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误
    // 但有时候最小距离非常小，设置一个经验值作为下限

    for (int i = 0; i < descriptor_pre_matches.size(); i++)
    {
        if (descriptor_pre_matches[i].distance <= max(2*min_dist, 30.0))
            matches_optimized.push_back(descriptor_pre_matches[i]);
    }

    //-- 第五步：绘制匹配结果
    cv::Mat show_descriptor_pre_matches;
    cv::Mat show_matches_optimized;
    cv::drawMatches(
        img_1, keypoints_1, img_2, keypoints_2, 
        descriptor_pre_matches, show_descriptor_pre_matches);
    cv::drawMatches(
        img_1, keypoints_1, img_2, keypoints_2, 
        matches_optimized, show_matches_optimized
    );
    cv::imshow("Descriptor pre match", show_descriptor_pre_matches);
    cv::waitKey(0);
    cv::imshow("Matches optimized", show_matches_optimized);
    cv::waitKey(0);
}