#include <libdistrect.hpp>
#include <algorithm>
#include <limits>

using namespace std;

namespace distrect
{
DistortionRectifier::DistortionRectifier()
{
    m_matlabEngine = matlab::engine::startMATLAB();
}

DistortionRectifier::~DistortionRectifier()
{
    matlab::engine::terminateEngineClient();
}

void DistortionRectifier::setImage(const cv::Mat image)
{
    mSetImage(image);
}

void DistortionRectifier::setImage(const string filePath)
{
    if (!filePath.empty())
    {
        cv::Mat tmpImage = cv::imread(filePath);
		cv::resize(tmpImage, tmpImage, cv::Size(),0.5, 0.5);
		mSetImage(tmpImage);
    }
}

cv::Mat DistortionRectifier::getCurImage()
{
    return m_curImage.clone();
}

cv::Mat DistortionRectifier::getCurGrayImage()
{
    return m_curGrayImage.clone();
}

void DistortionRectifier::mSetImage(cv::Mat image)
{
    if (image.empty())
    {
        cout << "empty image found. not setting it." << endl;
        return;
    }

    // set curImage
    m_curImage = image.clone();

    // set grayImage
    m_curGrayImage = image.clone();
    if (m_curGrayImage.channels() > 1)
    {
        cv::cvtColor(m_curGrayImage, m_curGrayImage, cv::COLOR_BGR2GRAY, 1);
    }
}

matlab::data::Array DistortionRectifier::getMatlabImage(const cv::Mat &grayImage)
{
    if (grayImage.channels() > 1)
    {
        throw std::runtime_error("image must be single channel");
    }

    switch (grayImage.depth())
    {
    case CV_8U:
        return mCvToMatlabCopy<uint8_t>(grayImage);
    case CV_8S:
        return mCvToMatlabCopy<int8_t>(grayImage);
    case CV_16U:
        return mCvToMatlabCopy<uint16_t>(grayImage);
    case CV_16S:
        return mCvToMatlabCopy<int16_t>(grayImage);
    case CV_32S:
        return mCvToMatlabCopy<int32_t>(grayImage);
    case CV_32F:
        return mCvToMatlabCopy<float>(grayImage);
    case CV_64F:
        return mCvToMatlabCopy<double>(grayImage);
    default:
        throw std::runtime_error("image data type not supported");
    }
}

LineSegmentList DistortionRectifier::getLineSegments()
{
    if (m_curImage.empty() || m_curGrayImage.empty())
    {
        throw runtime_error("nothing to do. image is not set or empty.");
    }

    vector<matlab::data::Array> args;
    args.push_back(getMatlabImage(m_curGrayImage));
    args.push_back(m_arrayFactory.createScalar<int>(m_curGrayImage.rows));
    args.push_back(m_arrayFactory.createScalar<int>(m_curGrayImage.cols));
    matlab::data::TypedArray<double> temp = m_matlabEngine->feval(
        matlab::engine::convertUTF8StringToUTF16String("EDPFLinesmex"),
        args);

    size_t noLines = temp.getDimensions()[1];
    vector<ILineSegment> lineSegments;
    for (size_t i = 0; i < noLines; i++)
    {
        ILineSegment segment;
        segment.a = temp[0][i];
        segment.b = temp[1][i];
        segment.invert = (temp[2][i] == 1.0) ? true : false;
        segment.sx = temp[4][i];
        segment.sy = temp[3][i];
        segment.ex = temp[6][i];
        segment.ey = temp[5][i];
        segment.segmentNo = (int)temp[7][i];

        lineSegments.push_back(segment);
    }

    LineSegmentList segments;

    int curEdgeSeg = -1;
    int curIndex = -1;
    for (size_t i = 0; i < (lineSegments.size() - 1); i++)
    {
        ILineSegment segment(lineSegments[i]);
        if (segment.segmentNo != curEdgeSeg)
        {
            if (segment.segmentNo != lineSegments[i + 1].segmentNo)
            {
                continue;
            }

            curEdgeSeg = segment.segmentNo;
            vector<ILineSegment> tempSegment;
            tempSegment.push_back(segment);
            segments.push_back(tempSegment);
            curIndex++;
        }
        else
        {
            segments[curIndex].push_back(segment);
        }
    }

    return segments;
}

LineSegmentList DistortionRectifier::filterLineSegments(LineSegmentList segments, double lengthThres, double radDistThres)
{
    if (segments.empty())
    {
        throw runtime_error("empty list of line segments found");
    }

    if (m_curGrayImage.empty())
    {
        throw runtime_error("image is not set");
    }

    cv::Point3_<double> imageCenter(m_curGrayImage.cols / 2.0f, m_curGrayImage.rows / 2.0f, 1.0);
    LineSegmentList outLineGroup;
    for (auto cell : segments)
    {
        vector<ILineSegment> newLineGroup;
        for (auto curLine : cell)
        {
            double dist = pow(curLine.sx - curLine.ex, 2.0) + pow(curLine.sy - curLine.ey, 2.0);
            if (dist < (lengthThres * lengthThres))
            {
                continue;
            }

            cv::Point3_<double> vecA(curLine.sx, curLine.sy, 1.0);
            cv::Point3_<double> vecB(curLine.ex, curLine.ey, 1.0);
            cv::Point3_<double> line = vecA.cross(vecB);

            double norm = sqrt(
                pow(line.x, 2.0) +
                pow(line.y, 2.0));

            line = line / norm;
            double distFromCent = line.dot(imageCenter);
            if (distFromCent < 0.0)
            {
                distFromCent *= -1.0;
            }

            if (distFromCent < radDistThres)
            {
                continue;
            }

            newLineGroup.push_back(curLine);
        }
        if (newLineGroup.size() > 1)
        {
            outLineGroup.push_back(newLineGroup);
        }
    }

    return outLineGroup;
}

LineSegmentList DistortionRectifier::groupLineSegments(LineSegmentList segments, double distThres, double angleThres)
{
    if (segments.empty())
    {
        throw runtime_error("empty line segment list found");
    }

    LineSegmentList outLineGroup;
    for (int groupIx = 0; groupIx < segments.size(); groupIx++)
    {
        vector<double> errors;
        for (int segIx = 0; segIx < segments[groupIx].size(); segIx++)
        {
            int nextSegIx = segIx + 1;
            if (nextSegIx == segments[groupIx].size())
            {
                nextSegIx = 0;
            }

            errors.push_back(mGetLineError(
                segments[groupIx][segIx],
                segments[groupIx][nextSegIx]));
        }

        vector<int> usedLines;
        while (true)
        {
            double minError = 0.0;
            int minErrorId = 0;
            for (int i = 0; i < errors.size(); i++)
            {
                if (i == 0)
                {
                    minError = errors[i];
                    minErrorId = i;
                    continue;
                }

                if (errors[i] < minError)
                {
                    minError = errors[i];
                    minErrorId = i;
                }
            }

            if (minError > distThres)
            {
                break;
            }

            int nextMinErrorId = minErrorId + 1;
            if (nextMinErrorId == errors.size())
            {
                nextMinErrorId = 0;
            }

            vector<int> seedSeg({minErrorId, nextMinErrorId});
            vector<int> groupIndices(seedSeg);

            errors[seedSeg[0]] = numeric_limits<double>::max();

            double ang1 = mGetLineSegmentAngle(segments[groupIx][seedSeg[0]]);
            double ang2 = mGetLineSegmentAngle(segments[groupIx][seedSeg[1]]);

            double angDiff = mGetDifferenceOfAngles(ang1, ang2);

            if (angDiff > angleThres)
            {
                continue;
            }

            usedLines.push_back(seedSeg[0]);
            usedLines.push_back(seedSeg[1]);

            int prevLineSegId = seedSeg[0] - 1;
            if (prevLineSegId < 0)
            {
                prevLineSegId = ((errors.size() - 1) > 0) ? (int)(errors.size() - 1) : 0;
            }

            if (find(usedLines.begin(), usedLines.end(), prevLineSegId) == usedLines.end())
            {
                double ang0 = mGetLineSegmentAngle(segments[groupIx][prevLineSegId]);
                angDiff = mGetDifferenceOfAngles(ang1, ang0);

                if (angDiff < angleThres)
                {
                    if (errors[prevLineSegId] < distThres)
                    {
                        errors[prevLineSegId] = numeric_limits<double>::max();
                        usedLines.push_back(prevLineSegId);
                        vector<int> tmpIndices({prevLineSegId});
                        tmpIndices.insert(tmpIndices.end(), groupIndices.begin(), groupIndices.end());
                        groupIndices.clear();
                        groupIndices.insert(groupIndices.end(), tmpIndices.begin(), tmpIndices.end());
                    }
                }
            }

            int nextLineSegId = seedSeg[1] + 1;
            if (nextLineSegId == errors.size())
            {
                nextLineSegId = 0;
            }

            if (find(usedLines.begin(), usedLines.end(), nextLineSegId) == usedLines.end())
            {
                double ang3 = mGetLineSegmentAngle(segments[groupIx][nextLineSegId]);
                angDiff = mGetDifferenceOfAngles(ang1, ang3);
                if (angDiff < angleThres)
                {
                    if (errors[nextLineSegId] < distThres)
                    {
                        errors[nextLineSegId] = numeric_limits<double>::max();
                        usedLines.push_back(nextLineSegId);

                        groupIndices.push_back(nextLineSegId);
                    }
                }
            }

            vector<ILineSegment> tmpGroup;
            for (auto ix : groupIndices)
            {
                tmpGroup.push_back(segments[groupIx][ix]);
            }

            outLineGroup.push_back(tmpGroup);
        }
    }
    return outLineGroup;
}

double DistortionRectifier::mGetLineError(ILineSegment seg1, ILineSegment seg2)
{
    LineSegmentList segments;
    segments.push_back(vector<ILineSegment>({seg1, seg2}));
    segments.push_back(vector<ILineSegment>({seg2, seg1}));
    vector<double> errors;
    for (auto cell : segments)
    {
        cv::Point3_<double> tmpA(cell[0].sx, cell[0].sy, 1.0);
        cv::Point3_<double> tmpB(cell[0].ex, cell[0].ey, 1.0);
        cv::Point3_<double> line1 = tmpA.cross(tmpB);
        double norm = sqrt(
            pow(line1.x, 2.0) +
            pow(line1.y, 2.0));
        line1 = line1 / norm;
        cv::Point3_<double> tmpA2(cell[1].sx, cell[1].sy, 1.0);
        cv::Point3_<double> tmpB2(cell[1].ex, cell[1].ey, 1.0);
        double dist1 = line1.dot(tmpA2);
        double dist2 = line1.dot(tmpB2);
        double error1 = (dist1 > dist2) ? dist2 : dist1;
        errors.push_back(error1);
    }

    sort(errors.begin(), errors.end());

    return errors[0];
}

double DistortionRectifier::mGetDifferenceOfAngles(double ang1, double ang2)
{
    double diff = ang1 - ang2;
    if (diff < 0.0)
    {
        diff *= -1.0;
    }
    if (diff > 180.0)
    {
        diff = 360.0 - diff;
    }

    return diff;
}

double DistortionRectifier::mGetLineSegmentAngle(ILineSegment seg)
{
    return radtodegree(
        atan2(
            (seg.ey - seg.sy),
            (seg.ex - seg.sx)));
}

matlab::data::CellArray DistortionRectifier::mGetLineSegments(LineSegmentList segments)
{
    if (segments.empty())
    {
        throw runtime_error("empty list of line segments");
    }

    matlab::data::ArrayDimensions dim;
    matlab::data::CellArray rv = m_arrayFactory.createCellArray({1, segments.size()});

    for (int j = 0; j < segments.size(); j++)
    {
        vector<ILineSegment> cell = segments[j];
        matlab::data::TypedArray<double> cellArray = m_arrayFactory.createArray<double>({cell.size(), 4});
        for (int i = 0; i < cell.size(); i++)
        {
            cellArray[i][0] = cell[i].sx;
            cellArray[i][1] = cell[i].sy;
            cellArray[i][2] = cell[i].ex;
            cellArray[i][3] = cell[i].ey;
        }
        rv[j] = cellArray;
    }

    return rv;
}

LineSegmentList DistortionRectifier::selectLineSegmentGroups(LineSegmentList segments)
{
    if (m_curGrayImage.empty())
    {
        throw runtime_error("image not set. please set image and find the line groups first.");
    }

    LineSegmentList lineGroups(segments);
    matlab::data::Array mImage = getMatlabImage(m_curGrayImage);

    while (true)
    {
        if (lineGroups.size() <= MIN_NUM_OF_SELECTED_LINE_GROUPS)
        {
            break;
        }

        matlab::data::CellArray mLineGroups = mGetLineSegments(lineGroups);
        matlab::data::TypedArray<double> minErrorT = m_matlabEngine->feval(
            matlab::engine::convertUTF8StringToUTF16String("GetFMin"),
            vector<matlab::data::Array>({mImage, mLineGroups}));

        double minError = minErrorT[2][0]; // the 3rd row is the fval

        LineSegmentList origLineGroup(lineGroups);

        int indToEliminate = -1;
        for (int i = 0; i < origLineGroup.size(); i++)
        {
            lineGroups.clear();

            for (int j = 0; j < origLineGroup.size(); j++)
            {
                if (j != i)
                {
                    lineGroups.push_back(origLineGroup[j]);
                }
            }

            matlab::data::CellArray tmpMLineGroups = mGetLineSegments(lineGroups);
            matlab::data::TypedArray<double> tmpErrorT = m_matlabEngine->feval(
                matlab::engine::convertUTF8StringToUTF16String("GetFMin"),
                vector<matlab::data::Array>({mImage, tmpMLineGroups}));

            double tmpError = tmpErrorT[2][0];
            if (tmpError < minError)
            {
                minError = tmpError;
                indToEliminate = i;
            }
        }

        if (indToEliminate < 0)
        {
            lineGroups.clear();
            lineGroups.insert(lineGroups.end(), origLineGroup.begin(), origLineGroup.end());
            break;
        }
        else
        {
            cout << "Eliminating line segment group #" << indToEliminate << endl;
            lineGroups.clear();
            for (int i = 0; i < origLineGroup.size(); i++)
            {
                if (i != indToEliminate)
                {
                    lineGroups.push_back(origLineGroup[i]);
                }
            }
        }
    }

    return lineGroups;
}

camera_props DistortionRectifier::getCameraParams(LineSegmentList segments)
{
    if (m_curGrayImage.empty())
    {
        throw runtime_error("image is not set. please set the image first.");
    }

    camera_props props;
    props.intrinsic_matrix = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
    props.intrinsic_matrix.at<float>(0, 0) = 1.0f;
    props.intrinsic_matrix.at<float>(0, 2) = float(m_curGrayImage.cols) / 2.0f;
    props.intrinsic_matrix.at<float>(1, 1) = 1.0f;
    props.intrinsic_matrix.at<float>(1, 2) = float(m_curGrayImage.rows) / 2.0f;
    props.intrinsic_matrix.at<float>(2, 2) = 1.0f;

    matlab::data::Array mImage = getMatlabImage(m_curGrayImage);
    matlab::data::CellArray mLineGroups = mGetLineSegments(segments);

    matlab::data::TypedArray<double>
        params = m_matlabEngine->feval(
            matlab::engine::convertUTF8StringToUTF16String("GetFMin"),
            vector<matlab::data::Array>({mImage,
                                         mLineGroups}));

    props.distortion_params = cv::Mat(1, 4, CV_32F, cv::Scalar(0.0));
    props.distortion_params.at<float>(0, 0) = (float)params[0][0];
    props.distortion_params.at<float>(0, 1) = (float)params[1][0];

    return props;
}

cv::Mat DistortionRectifier::undistort(camera_props props, double alpha)
{
    if (m_curImage.empty())
    {
        throw runtime_error("image is not set.");
    }
    if (alpha > UNDIST_FULL || alpha < UNDIST_VALID)
    {
        throw runtime_error("alpha must be between " + to_string(UNDIST_VALID) + " to " + to_string(UNDIST_FULL));
    }

    cv::Mat newCamMat = cv::getOptimalNewCameraMatrix(props.intrinsic_matrix, props.distortion_params, m_curGrayImage.size(), alpha);

    cv::Mat rv;
    //cv::undistort(m_curImage, rv, props.intrinsic_matrix, props.distortion_params);
    cv::undistort(m_curImage, rv, props.intrinsic_matrix, props.distortion_params, newCamMat);

    return rv;
}

cv::Mat DistortionRectifier::undistort()
{
    if (m_curImage.empty())
    {
        throw runtime_error("image is not set.");
    }

    LineSegmentList segments = getLineSegments();
    LineSegmentList filteredSegments = filterLineSegments(segments);
    LineSegmentList groupedSegments = groupLineSegments(filteredSegments);
    LineSegmentList finalSegments = selectLineSegmentGroups(groupedSegments);

    camera_props props = getCameraParams(finalSegments);

    return undistort(props);
}

} // namespace distrect