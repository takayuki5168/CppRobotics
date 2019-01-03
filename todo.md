- linksとjointの構造変える
    - DHパラメータはjoint単体のものではなく，joint+linkなのでは
    - linkはひとつのjointを持つ
    - linkが木構造を経てlinksになる
    
```cpp
auto links = Links({link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational)};
```