package main

import (
    "math/rand"
    "fmt"
    "image/color"
    "log"
    "os"
    "runtime/pprof"
    "flag"

    "github.com/hajimehoshi/ebiten/v2"
    "github.com/hajimehoshi/ebiten/v2/ebitenutil"
    "github.com/hajimehoshi/ebiten/v2/inpututil"
)


const (
    screenWidth  = 1000
    screenHeight = 1000
    particleSize = 3

    gridWidth    = 100
    gridHeight   = 100
    numParticles = gridWidth * gridHeight * 4
    picRatio     = 0.05
    flipRatio    = 1 - picRatio
    gravity      = 980
    timeStep     = 0.01

    widthScale = screenWidth / gridWidth
    heightScale = screenHeight / gridHeight
)

type ParticleSystem struct {
    positions [numParticles][2]float64
    velocities [numParticles][2]float64
}

type CellContent int

const (
    EMPTY CellContent = iota
    FLUID
    SOLID
)

type Grid struct {
    width, height int
    cellContents [gridWidth][gridHeight]CellContent
    cellPressures [gridWidth][gridHeight]float64
    cellDivergences [gridWidth][gridHeight]float64
    horizontalEdgeVelocities [gridWidth+1][gridHeight]float64
    verticalEdgeVelocities [gridWidth][gridHeight+1]float64
    horizontalEdgeVelocityWeights [gridWidth+1][gridHeight]float64
    verticalEdgeVelocityWeights [gridWidth][gridHeight+1]float64
    previousHorizontalEdgeVelocities [gridWidth+1][gridHeight]float64
    previousVerticalEdgeVelocities [gridWidth][gridHeight+1]float64
    particles ParticleSystem
}

func (g *Grid) Splat(x, y, vx, vy float64) {
    cellX := int(x)
    cellY := int(y)
    g.cellContents[cellX][cellY] = FLUID
    weightX := x - float64(cellX)
    weightY := y - float64(cellY)
    shiftedX := x - 0.5
    shiftedY := y - 0.5
    shiftedCellX := int(shiftedX)
    shiftedCellY := int(shiftedY)
    shiftedWeightX := shiftedX - float64(shiftedCellX)
    shiftedWeightY := shiftedY - float64(shiftedCellY)

    lowerLeftWeight := (1-weightX) * (1-shiftedWeightY)
    lowerRightWeight := weightX * (1-shiftedWeightY)
    upperLeftWeight := (1-weightX) * shiftedWeightY
    upperRightWeight := weightX * shiftedWeightY

    g.horizontalEdgeVelocities[cellX][shiftedCellY] += upperLeftWeight * vx
    g.horizontalEdgeVelocities[cellX+1][shiftedCellY] += upperRightWeight * vx
    g.horizontalEdgeVelocities[cellX][shiftedCellY+1] += lowerLeftWeight * vx
    g.horizontalEdgeVelocities[cellX+1][shiftedCellY+1] += lowerRightWeight * vx
    g.horizontalEdgeVelocityWeights[cellX][shiftedCellY] += upperLeftWeight
    g.horizontalEdgeVelocityWeights[cellX+1][shiftedCellY] += upperRightWeight
    g.horizontalEdgeVelocityWeights[cellX][shiftedCellY+1] += lowerLeftWeight
    g.horizontalEdgeVelocityWeights[cellX+1][shiftedCellY+1] += lowerRightWeight

    lowerLeftWeight = (1-shiftedWeightX) * (1-weightY)
    lowerRightWeight = shiftedWeightX * (1-weightY)
    upperLeftWeight = (1-shiftedWeightX) * weightY
    upperRightWeight = shiftedWeightX * weightY

    g.verticalEdgeVelocities[shiftedCellX][cellY] += upperLeftWeight * vy
    g.verticalEdgeVelocities[shiftedCellX+1][cellY] += upperRightWeight * vy
    g.verticalEdgeVelocities[shiftedCellX][cellY+1] += lowerLeftWeight * vy
    g.verticalEdgeVelocities[shiftedCellX+1][cellY+1] += lowerRightWeight * vy
    g.verticalEdgeVelocityWeights[shiftedCellX][cellY] += upperLeftWeight
    g.verticalEdgeVelocityWeights[shiftedCellX+1][cellY] += upperRightWeight
    g.verticalEdgeVelocityWeights[shiftedCellX][cellY+1] += lowerLeftWeight
    g.verticalEdgeVelocityWeights[shiftedCellX+1][cellY+1] += lowerRightWeight
}

func (g *Grid) ClampSolidCells() {
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            if g.cellContents[i][j] == SOLID {
                g.horizontalEdgeVelocities[i][j] = 0
                g.horizontalEdgeVelocities[i+1][j] = 0
                g.verticalEdgeVelocities[i][j] = 0
                g.verticalEdgeVelocities[i][j+1] = 0
            }
        }
    }
}

func (g *Grid) ParticlesToGrid() {
    var x, y, vx, vy float64
    g.previousHorizontalEdgeVelocities = g.horizontalEdgeVelocities
    g.previousVerticalEdgeVelocities = g.verticalEdgeVelocities
    g.horizontalEdgeVelocities = [gridWidth+1][gridHeight]float64{}
    g.verticalEdgeVelocities = [gridWidth][gridHeight+1]float64{}
    for i := 1; i < gridWidth - 1; i++ {
        for j := 1; j < gridHeight - 1; j++ {
            g.cellContents[i][j] = EMPTY
        }
    }
    for i := 0; i < numParticles; i++ {
        x = g.particles.positions[i][0]
        y = g.particles.positions[i][1]
        vx = g.particles.velocities[i][0]
        vy = g.particles.velocities[i][1]
        g.Splat(x, y, vx, vy)
    }
    for i := 0; i < gridWidth + 1; i++ {
        for j := 0; j < gridHeight; j++ {
            if g.horizontalEdgeVelocityWeights[i][j] != 0 {
                g.horizontalEdgeVelocities[i][j] /= g.horizontalEdgeVelocityWeights[i][j]
            }
        }
    }
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight + 1; j++ {
            if g.verticalEdgeVelocityWeights[i][j] != 0 {
                g.verticalEdgeVelocities[i][j] /= g.verticalEdgeVelocityWeights[i][j]
            }
        }
    }
}

func interpolateGridVelocities(
    x, y float64,
    horizontalEdgeVelocities *[gridWidth+1][gridHeight]float64,
    verticalEdgeVelocities *[gridWidth][gridHeight+1]float64) (float64, float64) {

    cellX := int(x)
    cellY := int(y)
    weightX := x - float64(cellX)
    weightY := y - float64(cellY)
    shiftedX := x - 0.5
    shiftedY := y - 0.5
    shiftedCellX := int(shiftedX)
    shiftedCellY := int(shiftedY)
    shiftedWeightX := shiftedX - float64(shiftedCellX)
    shiftedWeightY := shiftedY - float64(shiftedCellY)
    vx := (1-weightX) * (1-shiftedWeightY) * horizontalEdgeVelocities[cellX][shiftedCellY] +
          weightX * (1-shiftedWeightY) * horizontalEdgeVelocities[cellX+1][shiftedCellY] +
          (1-weightX) * shiftedWeightY * horizontalEdgeVelocities[cellX][shiftedCellY+1] +
          weightX * shiftedWeightY * horizontalEdgeVelocities[cellX+1][shiftedCellY+1]
    vy := (1-shiftedWeightX) * (1-weightY) * verticalEdgeVelocities[shiftedCellX][cellY] +
          shiftedWeightX * (1-weightY) * verticalEdgeVelocities[shiftedCellX+1][cellY] +
          (1-shiftedWeightX) * weightY * verticalEdgeVelocities[shiftedCellX][cellY+1] +
          shiftedWeightX * weightY * verticalEdgeVelocities[shiftedCellX+1][cellY+1]
    return vx, vy
}

func sign(x int) int {
    if x < 0 {
        return -1
    } else if x > 0 {
        return 1
    } else {
        return 0
    }
}

func abs(x int) int {
    if x < 0 {
        return -x
    } else {
        return x
    }
}

func (g *Grid) UpdateParticles() {
    for i := 0; i < numParticles; i++ {
        vx := g.particles.velocities[i][0]
        vy := g.particles.velocities[i][1]
        x := g.particles.positions[i][0] + vx * timeStep
        y := g.particles.positions[i][1] + vy * timeStep
        g.particles.positions[i][0] = x
        g.particles.positions[i][1] = y
        cellX := int(x)
        cellY := int(y)
        if cellX < 0 || cellX >= gridWidth || cellY < 0 || cellY >= gridHeight {
            print("out of bounds: ", cellX, " ", cellY, "\n")
            print("position: ", x, " ", y, "\n")
            print("velocity: ", vx, " ", vy, "\n")
        }
        if g.cellContents[cellY][cellX] == SOLID {
            vx := g.particles.velocities[i][0]
            vy := g.particles.velocities[i][1]
            previousCellX := int(x - vx * timeStep)
            previousCellY := int(y - vy * timeStep)
            cellDx := cellX - previousCellX
            cellDy := cellY - previousCellY
            xDirection := sign(cellDx)
            yDirection := sign(cellDy)
            for cellShiftX := 0; cellShiftX <= abs(cellDx); cellShiftX++ {
                for cellShiftY := 0; cellShiftY <= abs(cellDy); cellShiftY++ {
                    if g.cellContents[cellY+cellShiftY][cellX+cellShiftX] != SOLID {
                        newCellX := cellX + cellShiftX * xDirection
                        newCellY := cellY + cellShiftY * yDirection
                        if xDirection == -1 {
                            g.particles.positions[i][0] = float64(newCellX) + .99
                        } else if xDirection == 1 {
                            g.particles.positions[i][0] = float64(newCellX)
                        }
                        if yDirection == -1 {
                            g.particles.positions[i][1] = float64(newCellY) + .99
                        } else if yDirection == 1 {
                            g.particles.positions[i][1] = float64(newCellY)
                        }
                        return
                    }
                }
            }

        }
    }
}

func (g *Grid) ApplyGravity() {
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight + 1; j++ {
            g.verticalEdgeVelocities[i][j] -= gravity * timeStep
        }
    }
}

func (g *Grid) MakeIncompressible() {
    // TODO
}

func (g *Grid) GridToParticles() {
    var x, y, vx, vy, vOldX, vOldY, vPicX, vPicY, vFlipX, vFlipY float64
    for i := 0; i < numParticles; i++ {
        x = g.particles.positions[i][0]
        y = g.particles.positions[i][1]
        vx = g.particles.velocities[i][0]
        vy = g.particles.velocities[i][1]
        vOldX, vOldY = interpolateGridVelocities(
            x, y,
            &g.previousHorizontalEdgeVelocities,
            &g.previousVerticalEdgeVelocities)
        vFlipX, vFlipY = interpolateGridVelocities(
            x, y,
            &g.horizontalEdgeVelocities,
            &g.verticalEdgeVelocities)
        vPicX = vx + vFlipX - vOldX
        vPicY = vy + vFlipY - vOldY
        g.particles.velocities[i][0] = flipRatio * vFlipX + picRatio * vPicX
        g.particles.velocities[i][1] = flipRatio * vFlipY + picRatio * vPicY
    }
}


func (g *Grid) Update() {
    g.ParticlesToGrid()
    g.ApplyGravity()
    g.ClampSolidCells()
    g.MakeIncompressible()
    g.GridToParticles()
    g.UpdateParticles()
}

var (
    particleSprite = ebiten.NewImage(particleSize, particleSize)
    cellSprite = map[CellContent]*ebiten.Image{
        EMPTY: ebiten.NewImage(int(widthScale), int(heightScale)),
        FLUID: ebiten.NewImage(int(widthScale), int(heightScale)),
        SOLID: ebiten.NewImage(int(widthScale), int(heightScale)),
    }
)

func init() {
    particleSprite.Fill(color.White)
    cellSprite[EMPTY].Fill(color.Black)
    cellSprite[FLUID].Fill(color.RGBA{0, 0, 255, 255})
    cellSprite[SOLID].Fill(color.RGBA{100, 100, 100, 255})
}

type Game struct {
    grid  *Grid
    keys  []ebiten.Key
}

func NewGrid(width, height int) *Grid {
    var g Grid

    // set cells on the edge to solid
    for i := 0; i < width; i++ {
        g.cellContents[0][i] = SOLID
        g.cellContents[height-1][i] = SOLID
    }
    for i := 0; i < height; i++ {
        g.cellContents[i][0] = SOLID
        g.cellContents[i][width-1] = SOLID
    }

    usableWidth := width - 2
    usableHeight := height - 2

    for i := 0; i < numParticles; i++ {
        g.particles.positions[i] = [2]float64{rand.Float64() * float64(usableWidth) + 1,
                                             rand.Float64() * float64(usableHeight) + 1}
        g.particles.velocities[i] = [2]float64{0, 0}
    }

    return &g
}

func NewGame() *Game {
    return &Game{
        grid: NewGrid(gridWidth, gridHeight),
        keys: []ebiten.Key{},
    }
}

func (g *Game) Update() error {
    g.keys = inpututil.AppendPressedKeys(g.keys[:0])
    if ebiten.IsKeyPressed(ebiten.KeyQ) {
        return ebiten.Termination
    }
    g.grid.Update()
    return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
    screen.Fill(color.Black)

    op := &ebiten.DrawImageOptions{}
    op.ColorScale.Scale(200.0/255.0, 200.0/255.0, 200.0/255.0, 1)

    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            x := float64(i) * widthScale
            y := float64(gridHeight - j - 1) * heightScale
            op.GeoM.Reset()
            op.GeoM.Translate(x, y)
            if g.grid.cellContents[i][j] == SOLID {
                screen.DrawImage(cellSprite[SOLID], op)
            } else if g.grid.cellContents[i][j] == FLUID {
                screen.DrawImage(cellSprite[FLUID], op)
            } else {
                screen.DrawImage(cellSprite[EMPTY], op)
            }
        }
    }

    for i := 0; i < numParticles; i++ {
        x := g.grid.particles.positions[i][0] * widthScale
        y := (gridHeight - g.grid.particles.positions[i][1]) * heightScale
        op.GeoM.Reset()
        op.GeoM.Translate(x, y)
        screen.DrawImage(particleSprite, op)

    }

    ebitenutil.DebugPrint(screen, fmt.Sprintf("TPS: %0.2f", ebiten.ActualTPS()))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
    return screenWidth, screenHeight
}

func main() {
    var (
        cpuprofile = flag.String("cpuprofile", "", "write cpu profile to `file`")
    )
    flag.Usage = func() {
        fmt.Fprintf(flag.CommandLine.Output(),
            "Usage: go-flip [-cpuprofile <file>]\n")
        flag.PrintDefaults()
    }
    flag.Parse()
    if *cpuprofile != "" {
        f, err := os.Create(*cpuprofile)
        if err != nil {
            log.Fatal("could not create CPU profile: ", err)
        }
        defer f.Close()
        if err := pprof.StartCPUProfile(f); err != nil {
            log.Fatal("could not start CPU profile: ", err)
        }
        defer pprof.StopCPUProfile()
    }

    ebiten.SetWindowSize(screenWidth*1.2, screenHeight*1.2)
    ebiten.SetWindowTitle("Fluid Simulation")
    game := NewGame()
    if err := ebiten.RunGame(game); err != nil {
        log.Fatal(err)
    }
}
