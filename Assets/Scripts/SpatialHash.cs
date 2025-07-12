using System.Collections.Generic;
using Unity.Mathematics;


public class SpatialHash
{
    private readonly float _cellSize;
    private readonly Dictionary<int3, List<int>> _cells;

    public SpatialHash(float cellSize) 
    { 
        _cellSize = cellSize;
        _cells = new Dictionary<int3, List<int>>();
    }

    public void Add(float3 position, int index)
    {
        int3 cell = NewCell(position, _cellSize);
        if (!_cells.TryGetValue(cell, out var list))
        {
            list = new List<int>();
            _cells.Add(cell, list);
        }
        list.Add(index);
    }

    public IEnumerable<int> Query(float3 position, float radius)
    {
        int3 minCell = NewCell(position - radius, _cellSize);
        int3 maxCell = NewCell(position + radius, _cellSize);

        for (int x = minCell.x; x <= maxCell.x; x++)
        {
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int z = minCell.z; z <= maxCell.z; z++)
                {
                    if (_cells.TryGetValue(new int3(x, y, z), out var list))
                    {
                        foreach (int idx in list)
                        {
                            yield return idx;
                        }
                    }
                }
            }
        }
    }

    int3 NewCell(float3 position, float size)
    {
        return new int3(
            (int)math.floor(position.x / size),
            (int)math.floor(position.y / size),
            (int)math.floor(position.z / size)
        );
    }
}
