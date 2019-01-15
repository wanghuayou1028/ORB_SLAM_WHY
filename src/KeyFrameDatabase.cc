/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size()); // 将mvInvertedFile的大小设置为单词数目
}

// class BowVector: 
//	public std::map<WordId, WordValue>
// 每个BowVector是一个映射，存放的是word的ID和value
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);  // 一种BOW向量会有很多个关键帧
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

//函数的三要素是：函数返回值类型，函数名称，函数参数
//函数的返回值是装有关键帧指针的vector
//该函数是类KeyFrameDatabase的成员函数,函数名是DetectLoopCandidate
//该函数的参数分别是KeyFrame类型的指针变量pKF和最小得分
//函数步骤：
//第一步：与pKF有共视单词的pKF1，且不能是与pKF在covisibility graph中与pKF直接相连的关键帧
//第二步：共视单词数必须大于minCommonWords的才可以留下 pKF2
//第三步：pKF2中与pKF的BOw得分必须大于minScore的才可以留下 pKF3
//第四步：pKF3要与自己最亲密的“朋友”PK与pKF之间的共视单词数，留下的记为pKF4
//第五步：pKF4中那些与pKF得分大于minScoreToRetain的才可以最终留下来 记为pKF5;
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    //set有自动排序功能，不能直接存取元素
    //list不可以随机存取元素
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);
        // 对当前帧的每个单词找出对应的所有关键帧，且这些关键帧是当前关键帧的共视关键帧，则将这些关键帧存入lKFsSharingWords
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first]; //找到这个单词所对应的所有关键帧
            // 遍历该单词对应的所有的关键帧
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId) //为了不对同一个关键帧进行重复检测，这里打一个标签
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi)) //如果和当前关键帧相链接的关键帧中不包含pKFi???这里是不是写反了
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++; //如果当前帧与该帧每共有一个单词，则加一，当前帧与这个关键帧共有的单词数
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    //IScoreAndMatch变量存放与pKF有共视关系的关键帧以及两者之间的得分
    //pair<float, KeyFrame>pair的使用是将两者捆绑到一起存储
    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 计算具有最多相同单词的数量
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f; // 设置阈值

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords) // 提取具有足够共视单词的关键帧
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore) // minScore是传入函数的参数， 保证分数大于minScore
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 获取当前关键帧最佳共视10个关键帧的累计分数和最佳分数关键帧，存放在lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        // 获取与pKFi最佳共视的10个关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // 找出累计分数大于minScoreToRetain的所有候选关键帧
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

/*
 *　函数功能：从关键帧库中找出与当前帧匹配最好的候选关键帧序列
 *　函数输入：当前帧
 *　函数输出：关键帧库中与当前帧匹配最好的候选关键帧序列
 *　几个变量含义：
 *　１．mBowVec:Bow向量，里面存储的是从BoW词袋中查询到的“单词”
 *　２．mvInvertedFile：是一个向量，里面存放的是一个个单词，每个单词对应一个list链表，每个链表里存放的是拥有该单词的关键帧
 * 即，每一个单词都对应着所有看到该单词的关键帧
 * ３.pKFi->mnRelocWords:该关键帧与当前帧所共有的单词数
 *
 * 实现过程：首先找到具有相同关键词的关键帧，然后找出具有大于一定数量相同单词的关键帧，
 * 并且计算当前帧与该关键帧的得分，继续对这些关键帧找到10个最优的共视关键帧，累计与当前帧的分数，
 * 然后将累计分数大于0.75倍的累计分数的关键帧作为候选重定位关键帧
 */
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    list<KeyFrame*> lKFsSharingWords; // 与当前帧具有公共单词的所有关键帧

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        //遍历所有单词
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first]; //找到这个单词所对应的所有关键帧

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId) //为了不对同一个关键帧进行重复检测，这里打一个标签
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++; //如果当前帧与该帧每共有一个单词，则加一，当前帧与这个关键帧共有的单词数
            }
        }
    }
    if(lKFsSharingWords.empty()) //如果关键帧库里所有的关键帧与当前帧都没有公共单词，则返回空
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords; //遍历所有与当前帧具有公共单词的关键帧，并找与当前帧具有的最大公共单词数
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    //找出共有单词数在minCommonWords与maxCommonWords之间的关键帧，作为“得分较高的关键帧”，
    //并且计算当前帧与该关键帧的得分，将得分与该关键帧组成pair存到lScoreAndMatch中
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    //这一个应该是不会发生的，因为至少有一个关键帧与当前帧的共有单词数符合上边的阈值范围，就是共有单词数最多的那个关键帧
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 遍历得分较高的关键帧所对应的10个最佳共视关键帧，这样lScoreAndMatch序列中“被共视”的越多的关键帧将越有可能被选为候选关键帧
    // 这个函数还可以进行处理，因为函数首先在选取了每个关键帧的10个共视关键帧，然后将这些11个关键帧的分数加在一起，最后会选择出一个分数最高的11个关键帧组合
    // 但是，在每个关键帧处理过程中，如果他的１０个共视关键帧中有一个关键帧的分数比它自身大，就设置pBestKF为这个分数比较大的关键帧，这可能会出现重复的现象，后面也会对这个
    // 共识关键帧进行处理，如果他的周围１０个共视关键帧没有比他更大的分数，那么这个共视关键帧就会两次存到lAccScoreAndMatch中，且这两次具有不同的累计分数
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        // 获取这个关键帧的10个最佳共视关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        // 遍历该关键帧对应的10个最佳共视关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId) // 如果该关键帧不在lKFsSharingWords中，则无需再处理
                continue;

            // 该关键帧如果在lKFsSharingWords中
            accScore+=pKF2->mRelocScore; // 将pKFi的得分与他的共识关键帧得分累加
            if(pKF2->mRelocScore>bestScore) // 如果共视关键帧的得分超过pKFi，则将pKFi替换掉
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        // 这个变量中可能会出现相同的pBestKF
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 将lAccScoreAndMatch accScore中满足阈值的得分作为候选关键帧，返回
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
