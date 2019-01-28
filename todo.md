- linksとjointの構造変える
    - DHパラメータはjoint単体のものではなく，joint+linkなのでは
    - linkはひとつのjointを持つ
    - linkが木構造を経てlinksになる

- 基礎ヤコビ行列を使わない角速度ベクトルを用いたIKへ

```cpp
auto links = Links({link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational),
                    link(1, 1, 1, 1, JointType::Rotational)};
```