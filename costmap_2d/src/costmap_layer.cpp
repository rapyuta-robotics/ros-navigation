#include<costmap_2d/costmap_layer.h>

namespace costmap_2d
{

void CostmapLayer::touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
}

void CostmapLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area)
{
  std::vector<unsigned char*> grids = getTimedCharMaps();
  for(int x=0; x<(int)getSizeInCellsX(); x++){
    bool xrange = x>start_x && x<end_x;
    for(int y=0; y<(int)getSizeInCellsY(); y++){
      if((xrange && y>start_y && y<end_y)!=invert_area)
        continue;
      int index = getIndex(x,y);
      for (auto grid : grids)
      {
        if(grid[index]!=NO_INFORMATION){
          grid[index] = NO_INFORMATION;
        }
      }
    }
  }
}

void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
    extra_min_x_ = std::min(mx0, extra_min_x_);
    extra_max_x_ = std::max(mx1, extra_max_x_);
    extra_min_y_ = std::min(my0, extra_min_y_);
    extra_max_y_ = std::max(my1, extra_max_y_);
    has_extra_bounds_ = true;
}

void CostmapLayer::useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!has_extra_bounds_)
        return;

    *min_x = std::min(extra_min_x_, *min_x);
    *min_y = std::min(extra_min_y_, *min_y);
    *max_x = std::max(extra_max_x_, *max_x);
    *max_y = std::max(extra_max_y_, *max_y);
    extra_min_x_ = 1e6;
    extra_min_y_ = 1e6;
    extra_max_x_ = -1e6;
    extra_max_y_ = -1e6;
    has_extra_bounds_ = false;
}

// Update for static Layers only!!!
void CostmapLayer::updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  std::vector<unsigned char*> master_arrays = master_grid.getTimedCharMaps(); // Master costmap
  unsigned int span = master_grid.getSizeInCellsX();

  {
    for (int j = min_j; j < max_j; j++)
    {
      unsigned int it = j * span + min_i;
      for (int i = min_i; i < max_i; i++)
      {
        if (costmap_[it] == NO_INFORMATION){
          it++;
          continue;
        }
        unsigned char old_cost = master_grid.getCharMap()[it];
        if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
          for(unsigned char* master_array : master_arrays) 
            master_array[it] = costmap_[it];
        it++;
      }
    }
  }
}

void CostmapLayer::updateWithMaxTimed(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  std::vector<unsigned char*> master_arrays = master_grid.getTimedCharMaps(); // this is the master costmap
  unsigned int span = master_grid.getSizeInCellsX();

  for(uint n = 0; n < master_arrays.size(); ++n)
  {
  for (int j = min_j; j < max_j; j++)
    {
      unsigned int it = j * span + min_i;
      for (int i = min_i; i < max_i; i++)
      {
        if (timed_costmaps_[n][it] == NO_INFORMATION){
          it++;
          continue;
        }

        unsigned char old_cost = master_arrays[n][it];
        if (old_cost == NO_INFORMATION || old_cost < timed_costmaps_[n][it])
          master_arrays[n][it] = timed_costmaps_[n][it];
        it++;
      }
    }
  } 
}

void CostmapLayer::updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                           int max_i, int max_j)
{
  if (!enabled_)
    return;
  std::vector<unsigned char*> masters = master_grid.getTimedCharMaps();
  unsigned int span = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      for(unsigned char* master : masters)
        master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  std::vector<unsigned char*> masters = master_grid.getTimedCharMaps();
  unsigned int span = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] != NO_INFORMATION)
        for(unsigned char* master : masters)
          master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  std::vector<unsigned char*> master_arrays = master_grid.getTimedCharMaps();
  unsigned int span = master_grid.getSizeInCellsX();
  {
    for (int j = min_j; j < max_j; j++)
    {
      unsigned int it = j * span + min_i;
      for (int i = min_i; i < max_i; i++)
      {
        if (costmap_[it] == NO_INFORMATION){
          it++;
          continue;
        }
        unsigned char old_cost = master_arrays.front()[it];
        for(unsigned char* master_array : master_arrays)
        {
          if (old_cost == NO_INFORMATION)
            master_array[it] = costmap_[it];
          else
          {
            int sum = old_cost + costmap_[it];
            if (sum >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                master_array[it] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
            else
                master_array[it] = sum;
          }
        }
        it++;
        }
    }
  }
}
}  // namespace costmap_2d
