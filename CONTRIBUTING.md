## How to contribute this repository  

많은 사람들이 한번에 주석을 달 경우, 소스 충돌이 발생 할 가능성이 있어 규칙을 정하는 것이 좋을 것 같습니다.  

1. 주석을 달기 전에 Issue를 등록한다.  
> 자신이 Update할 함수가 어디서부터 어디까지인지 간단하게 작성해주시면 감사드리겠습니다.  

2. Issue number를 활용하여 PR을 보낸다.  
> 혹시 PR을 보내는 것에 대해 감이 잘 안오시는 분들은 [git 초보를 위한 풀리퀘스트(pull request) 방법](https://wayhome25.github.io/git/2017/07/08/git-first-pull-request-story/)를 참고해주세요!  
> PR을 보낼 때, repository에 최신 commit까지 반영을 해야 충돌을 피할 수 있습니다. (참고 : [[GitHub] fork repository 최신 버전으로 유지하기](https://jybaek.tistory.com/775))  
> 무작정 `git pull`을 사용할 시 자신이 썼던 주석 설명이 날라갈 수 있으니 주의해주세요!  
> commit messege를 작성할 시 Issue number를 활용해주세요. (참고 : [Git commit으로 Issue 종료하기(Closing issue with commit)](https://www.hahwul.com/2018/07/27/closing-git-issue-with-commit/))  

3. 확인 후, PR에 대한 merge를 진행하도록 하겠습니다.  

### 주석 style guide  

주석은 주로 `/include` 폴더 안에 있는 헤더 파일이나, `/src` 폴더 안에 있는 소스 파일에 해주시면 감사드리겠습니다.  

헤더 파일에는 함수에 대한 간략한 설명, 필요한 Parameters, return값들에 대한 설명이 들어가면 좋을 것 같습니다.  

함수를 설명할 때  
```
/*!
 * @brief 함수에 대한 간략한 설명.
 * @param 함수에 필요한 parameter1
 * @param 함수에 필요한 parameter2
 * @return 리턴용 값에 대한 설명
*/
```
소스 파일에는 Line by line으로 설명을 달아주세요.  

예를 들어,  

`Tracking.h`, `Tracking.cc`에 있는 `UpdateLocalMap()`에 대한 함수 설명을 진행한다고 할 때,  

`Tracking.h`에는  
```
/*!
 * @brief Local Map을 Update하는데 사용하는 함수.
 * @param None
 * @return None
*/
void UpdateLocalMap();
```  

`Tracking.cc`에는  
```
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // Visualization을 위한 Reference map point 지정
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Local key frame과 Local key points update를 수행
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}
```

주석을 달 때 참고한 Reference입니다.  
- [[C++] Kyuseo's C++ 독시젠을 활용한 주석 작성 스타일 가이드라인(규칙)](https://karfn84.tistory.com/entry/C-Kyuseos-C-%EB%8F%85%EC%8B%9C%EC%A0%A0%EC%9D%84-%ED%99%9C%EC%9A%A9%ED%95%9C-%EC%A3%BC%EC%84%9D-%EC%9E%91%EC%84%B1-%EC%8A%A4%ED%83%80%EC%9D%BC-%EA%B0%80%EC%9D%B4%EB%93%9C%EB%9D%BC%EC%9D%B8%EA%B7%9C%EC%B9%99)  
- [독시젠 A to Z](https://kieuns.com/doku.php?id=tool:doxygen)  

