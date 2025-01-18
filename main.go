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
    screenWidth  = 640
    screenHeight = 640
    particleSize = 3

    gridWidth    = 32
    gridHeight   = 32
    numParticles = gridWidth * gridHeight * 4
    initMinHeight = 0.5
    picRatio     = 0.05
    flipRatio    = 1 - picRatio

    gravity      = 980
    density      = .001
    timeStep     = 0.01
    resetDistance = 0.1 // distance from solid cells to reset particles entering them
    solverTolerance = 0.000001
    solverIterations = 1000

    widthScale = screenWidth / gridWidth
    heightScale = screenHeight / gridHeight

    neighbourCountMask = 0b00000111
    leftNeighbour = 1 << 3
    rightNeighbour = 1 << 4
    topNeighbour = 1 << 5
    bottomNeighbour = 1 << 6
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
    neighbourInfo [gridWidth][gridHeight]uint8
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
            if g.cellContents[i][j] == FLUID {
                g.cellContents[i][j] = EMPTY
            }
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

func (g *Grid) CalculateDivergences() {
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            g.cellDivergences[i][j] = 0
        }
    }
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            g.cellDivergences[i][j] = (g.verticalEdgeVelocities[i][j+1] - g.verticalEdgeVelocities[i][j] +
                                       g.horizontalEdgeVelocities[i+1][j] - g.horizontalEdgeVelocities[i][j]) *
                                        (-density) / timeStep
        }
    }
}

func (g *Grid) MultiplyNeighbourMatrix(vector *[gridWidth][gridHeight]float64,
                                       output *[gridWidth][gridHeight]float64) {
    var numNeighbours float64
    for i := 1; i < gridWidth - 1; i++ {
        for j := 1; j < gridHeight - 1; j++ {
            if g.cellContents[i][j] == FLUID {
                numNeighbours = float64(g.neighbourInfo[i][j] & neighbourCountMask)
                output[i][j] = vector[i][j] * numNeighbours
                if g.neighbourInfo[i][j] & leftNeighbour != 0 {
                    output[i][j] -= vector[i-1][j]
                }
                if g.neighbourInfo[i][j] & rightNeighbour != 0 {
                    output[i][j] -= vector[i+1][j]
                }
                if g.neighbourInfo[i][j] & bottomNeighbour != 0 {
                    output[i][j] -= vector[i][j-1]
                }
                if g.neighbourInfo[i][j] & topNeighbour != 0 {
                    output[i][j] -= vector[i][j+1]
                }
            }
        }
    }
}

func scalarProduct(a *[gridWidth][gridHeight]float64, b *[gridWidth][gridHeight]float64) float64 {
    var result float64
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            result += a[i][j] * b[i][j]
        }
    }
    return result
}

func (g *Grid) SolvePressure() {
    var residual [gridWidth][gridHeight]float64
    var stepDirection [gridWidth][gridHeight]float64
    var sigma, sigmaOld, relTolerance, alpha, beta float64
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            g.cellPressures[i][j] = 0
            stepDirection[i][j] = 0
            if g.cellContents[i][j] == FLUID {
                residual[i][j] = g.cellDivergences[i][j]
            }
        }
    }
    sigma = scalarProduct(&residual, &residual)
    relTolerance = solverTolerance * sigma
    for k := 0; k < solverIterations; k++ {
        g.MultiplyNeighbourMatrix(&g.cellDivergences, &stepDirection)
        alpha = sigma / scalarProduct(&g.cellDivergences, &stepDirection)
        if isnan(alpha) {
            print("solver diverged after ", k, " iterations\n")
            panic("solver diverged")
        }
        for i := 1; i < gridWidth - 1; i++ {
            for j := 1; j < gridHeight - 1; j++ {
                if g.cellContents[i][j] == FLUID {
                    g.cellPressures[i][j] += alpha * g.cellDivergences[i][j]
                    residual[i][j] -= alpha * stepDirection[i][j]
                }
            }
        }
        sigmaOld = sigma
        sigma = scalarProduct(&residual, &residual)
        if sigma < relTolerance {
            return
        }
        beta = sigma / sigmaOld
        for i := 1; i < gridWidth - 1; i++ {
            for j := 1; j < gridHeight - 1; j++ {
                if g.cellContents[i][j] == FLUID {
                    g.cellDivergences[i][j] = residual[i][j] + beta * g.cellDivergences[i][j]
                }
            }
        }
    }
    print("solver did not converge after ", solverIterations, " iterations\n")
}

func isnan(x float64) bool {
    return x != x
}

func (g *Grid) ClampPressure() {
    for i := 0; i < gridWidth; i++ {
        for j := 0; j < gridHeight; j++ {
            if g.cellContents[i][j] == FLUID {
                if g.cellPressures[i][j] < 0 {
                    g.cellPressures[i][j] = 0
                }
            }
        }
    }
}

func (g *Grid) ApplyPressure() {
    for i := 1; i < gridWidth; i++ {
        for j := 1; j < gridHeight; j++ {
            if g.cellContents[i][j] == SOLID {
                continue
            }
            if g.cellContents[i-1][j] == FLUID {
                g.horizontalEdgeVelocities[i][j] -= (g.cellPressures[i][j] - g.cellPressures[i-1][j]) *
                                                      timeStep / density
            } else if g.cellContents[i-1][j] == EMPTY {
                g.horizontalEdgeVelocities[i][j] -= g.cellPressures[i][j] * timeStep / density
            }
            if g.cellContents[i][j-1] == FLUID {
                g.verticalEdgeVelocities[i][j] -= (g.cellPressures[i][j] - g.cellPressures[i][j-1]) *
                                                    timeStep / density
            } else if g.cellContents[i][j-1] == EMPTY {
                g.verticalEdgeVelocities[i][j] -= g.cellPressures[i][j] * timeStep / density
            }
        }
    }
}

func (g *Grid) UpdateNeighbourInfo() {
    for i := 1; i < gridWidth - 1; i++ {
        for j := 1; j < gridHeight - 1; j++ {
            if g.cellContents[i][j] != FLUID {
                continue
            }
            g.neighbourInfo[i][j] = 4
            if g.cellContents[i-1][j] == SOLID {
                g.neighbourInfo[i][j] -= 1
            } else if g.cellContents[i-1][j] == FLUID {
                g.neighbourInfo[i][j] |= leftNeighbour
            }
            if g.cellContents[i+1][j] == SOLID {
                g.neighbourInfo[i][j] -= 1
            } else if g.cellContents[i+1][j] == FLUID {
                g.neighbourInfo[i][j] |= rightNeighbour
            }
            if g.cellContents[i][j-1] == SOLID {
                g.neighbourInfo[i][j] -= 1
            } else if g.cellContents[i][j-1] == FLUID {
                g.neighbourInfo[i][j] |= bottomNeighbour
            }
            if g.cellContents[i][j+1] == SOLID {
                g.neighbourInfo[i][j] -= 1
            } else if g.cellContents[i][j+1] == FLUID {
                g.neighbourInfo[i][j] |= topNeighbour
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
        if (cellX < 0 || cellX > gridWidth - 1 ||
            cellY < 0 || cellY > gridHeight - 1) ||
            g.cellContents[cellX][cellY] == SOLID {
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
                    newCellX := previousCellX + cellShiftX * xDirection
                    newCellY := previousCellY + cellShiftY * yDirection
                    if (newCellX < 0 || newCellX >= gridWidth ||
                        newCellY < 0 || newCellY >= gridHeight) {
                        continue
                    }
                    if g.cellContents[newCellX][newCellY] != SOLID {
                        if xDirection == -1 {
                            g.particles.positions[i][0] = float64(newCellX) + resetDistance
                        } else if xDirection == 1 {
                            g.particles.positions[i][0] = float64(newCellX) + 1 - resetDistance
                        }
                        if yDirection == -1 {
                            g.particles.positions[i][1] = float64(newCellY) + resetDistance
                        } else if yDirection == 1 {
                            g.particles.positions[i][1] = float64(newCellY) + 1 - resetDistance
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
    g.CalculateDivergences()
    g.SolvePressure()
    g.ClampPressure()
    g.ApplyPressure()
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
    g.UpdateNeighbourInfo()
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

    for i := 0; i < width; i++ {
        g.cellContents[i][0] = SOLID
        g.cellContents[i][height-1] = SOLID
    }
    for i := 0; i < height; i++ {
        g.cellContents[0][i] = SOLID
        g.cellContents[width-1][i] = SOLID
    }
    for  i:=int(float64(width)*0.4); i<int(float64(width)*0.9); i++ {
        for j:=int(float64(height)*0.4); j<int(float64(height)*0.6); j++ {
            g.cellContents[i][j] = SOLID
        }
    }

    usableWidth := width - 2
    usableHeight := height - 2

    for i := 0; i < numParticles; i++ {
        g.particles.positions[i] = [2]float64{(initMinHeight + (1-initMinHeight)*rand.Float64()) * float64(usableWidth) + 1,
                                             (initMinHeight + (1-initMinHeight)*rand.Float64()) * float64(usableHeight) + 1}
        g.particles.velocities[i] = [2]float64{1000000, 0}
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
