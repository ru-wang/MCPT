import random as rand


def Select(arr: list, k: int) -> int:
    piv = arr[k]
    l = []
    r = []
    for v in [arr[i] for i in range(len(arr)) if i != k]:
        l.append(v) if v < piv else r.append(v)
    if k == len(l):
        return piv
    return Select(l, k) if k < len(l) else Select(r, k - len(l) - 1)


def Split(arr: list) -> tuple[list, list]:
    if not arr:
        return [], []
    len_l = len(arr) // 2
    len_r = len(arr) - len_l
    median = Select(arr, len_l)

    l = []
    r = []
    m = []
    for v in arr:
        if v < median:
            l.append(v)
        elif v > median:
            r.append(v)
        else:
            m.append(v)
    r += m[len_l - len(l) :]
    l += m[: len_l - len(l)]
    return l, r


def Test(n: int):
    arr = []
    for i in range(n):
        arr.append(rand.randrange(n))

    sorted_arr = sorted(arr)
    l, r = Split(arr)

    print(f"N={len(sorted_arr)} {sorted_arr}")
    print(f"N={len(l)} {sorted(l)}")
    print(f"N={len(r)} {sorted(r)}")
    print()

    sorted_merged = sorted(l) + sorted(r)
    assert len(l) == len(r) or len(l) + 1 == len(r)
    assert sorted_merged == sorted_arr


def main():
    rand.seed(0)
    for n in range(0, 30):
        Test(n)


if __name__ == "__main__":
    main()
