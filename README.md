# Localização visual de robô móvel com AprilTag — LaR/UFBA

Este projeto utiliza uma câmera fixa de topo para estimar a posição e a orientação de um robô Husky no ambiente simulado do Laboratório de Robótica (LaR) da UFBA. Uma AprilTag instalada sobre o robô é identificada nas imagens, permitindo calcular sua pose no plano do laboratório (`x`, `y` e `yaw`).

O Husky percorre quatro pontos de navegação enquanto o sistema registra as poses estimadas. Ao final, essas informações são comparadas com as poses de referência do Gazebo para calcular os erros. **A pose do Gazebo é usada somente nessa comparação; não é fornecida ao estimador nem ao controlador.**

![Visão de topo do laboratório com o Husky e a AprilTag detectada](results/poster.png)

**Demonstração:** [vídeo da execução (MP4)](results/demo.mp4) · **Análise:** [resultados completos](results/RESULTADOS.md)

## Resultados obtidos

A execução registrada no repositório contém **2.521 poses comparadas** ao longo de aproximadamente **166,5 segundos** de simulação.

| Métrica | Posição | Orientação (yaw) |
|---|---:|---:|
| RMSE | 1,78 cm | 0,72° |
| Erro absoluto médio | 1,60 cm | 0,43° |
| Percentil 95 | 2,95 cm | 2,24° |
| Erro máximo | 3,90 cm | 3,24° |

![Trajetória estimada, referência e erros](results/errors.png)

Os valores acima correspondem à execução incluída na pasta [`results/`](results/). As métricas são calculadas apenas para as poses visuais detectadas e associadas temporalmente à referência. Trata-se de um resultado em **simulação**, não de uma validação experimental com câmera física.

## Requisitos

A execução principal foi preparada para **Linux com Docker Engine e Docker Compose**, usando um display X11 ou Xvfb. O contêiner inclui:

- Ubuntu 20.04 e ROS 1 Noetic;
- Gazebo Classic 11 e pacotes de simulação do Husky;
- OpenCV 4.2 com módulo `aruco`;
- bibliotecas Python e ferramentas para gerar o relatório e o vídeo.

> Mesmo com a interface do Gazebo desativada (`gui:=false`), a câmera simulada precisa de um ambiente de renderização.

## Instalação e execução

No terminal, execute:

```bash
git clone https://github.com/JoaoVitorCLima/lar_gazebo_joao_lima.git
cd lar_gazebo_joao_lima
chmod +x scripts/*.sh
./scripts/build.sh
./scripts/run_top_view.sh
```

O último comando inicia o cenário do LaR, o Husky com a AprilTag, a câmera de topo, o estimador visual, o controle por pontos de passagem, o avaliador e a gravação do vídeo.

Se houver problemas com a aceleração gráfica, tente a renderização por software:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/run_top_view.sh
```

Para executar sem abrir a interface do Gazebo, mas mantendo a renderização da câmera:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/run_top_view.sh gui:=false
```

### Acompanhar a execução

Com a simulação aberta, use outro terminal:

```bash
./scripts/shell.sh
```

Dentro do contêiner, você pode verificar o sinal de conclusão e visualizar a imagem anotada:

```bash
rostopic echo /top_view/done
rosrun image_view image_view image:=/top_view/annotated
```

Quando `/top_view/done` passar para `true`, o percurso terminou. Encerre o `roslaunch` com **Ctrl+C** para finalizar a avaliação e, depois, gere o relatório:

```bash
./scripts/shell.sh
# Dentro do contêiner:
rosrun lar_gazebo report_top_view.py /ws/src/lar_gazebo/results/poses.csv
```

O vídeo é salvo em `results/demo.mp4` e convertido para H.264 pelo contêiner. O GitHub pode exigir o download do arquivo para reproduzi-lo localmente. Aguarde o encerramento da gravação antes de interromper a simulação. **Novas execuções substituem** `results/poses.csv` e `results/demo.mp4`; copie esses arquivos se quiser guardar os resultados anteriores.

### Execução nativa (sem Docker)

Também é possível utilizar um workspace com ROS Noetic no Ubuntu 20.04. Clone o projeto dentro de `src/`, instale as dependências e compile:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin build
source devel/setup.bash
roslaunch lar_gazebo top_view.launch
```

Além dos pacotes Husky e Gazebo, são necessários `python3-opencv`, `python3-numpy`, `python3-matplotlib`, `ros-noetic-cv-bridge`, `ros-noetic-message-filters`, `ros-noetic-tf` e `ros-noetic-image-view`. Para gerar MP4 em H.264 na instalação nativa, instale também `ffmpeg`.

## Como funciona

O sistema foi dividido em três partes principais:

1. **Localização visual:** recebe a imagem da câmera, procura a AprilTag de ID 42 e estima a pose do Husky no referencial fixo `world`.
2. **Movimentação:** recebe somente a pose visual (`/top_view/pose`) e publica os comandos de velocidade para seguir os pontos definidos no código.
3. **Avaliação:** recebe a pose visual e o *ground truth* do Gazebo, associa as amostras pelo tempo e calcula os erros de posição e orientação.

### Câmera, marcador e referenciais

O experimento utiliza uma tag da família **36h11, ID 42**, montada horizontalmente sobre o Husky. O contorno preto utilizado na estimativa mede **0,40 m**. A câmera fica fixa em `(4,65; 3,00; 9,00)` m e produz imagens de **1600 × 1200 pixels a 15 Hz**. Seus parâmetros intrínsecos são lidos em `/overhead/camera_info`.

O referencial `world` é fixo no cenário, com `x` e `y` no plano do chão e `z` apontando para cima. A transformação entre a câmera e esse referencial é definida pela montagem conhecida no cenário. A altura do plano da tag é considerada fixa em **0,742 m**.

O processamento segue estas etapas:

1. Detectar os quatro cantos da AprilTag com OpenCV (`DICT_APRILTAG_36h11`) e selecionar o ID 42.
2. Usar `solvePnP` e o erro de reprojeção como verificação da geometria detectada.
3. Projetar os raios dos cantos no plano da tag, usando a calibração da câmera e a transformação para `world`.
4. Calcular a posição e a orientação da tag e compensar seu deslocamento em relação à base do Husky (`base_link`).

A estimativa considera o movimento no plano (`x`, `y`, `yaw`). A altura da tag e a posição da câmera são parâmetros fixos do cenário: **não são ajustados a partir da trajetória verdadeira**.

### Percurso do robô

O controlador percorre os seguintes pontos, em metros:

```text
(5.65, 3.00) → (5.65, 3.80) → (4.65, 3.80) → (4.65, 3.00)
```

A velocidade linear é limitada a **0,20 m/s**, e a angular, a **0,50 rad/s**. O robô para se a pose visual ficar desatualizada por mais de **0,5 s** e também ao concluir o percurso.

A rota é fixa e foi escolhida para a área livre deste cenário. Não há planejamento global nem desvio automático de obstáculos. Para observar sem movimentar o robô, execute o launch com `drive:=false`; para desativar o vídeo, use `record:=false`.

## Como os erros são calculados

A pose de referência é publicada pelo plugin do Gazebo no tópico `/evaluation/ground_truth`, que **somente o nó de avaliação utiliza**. Para comparar poses no mesmo instante, o avaliador usa o horário da imagem e interpola as amostras disponíveis do ground truth.

O erro de posição é a distância euclidiana entre as coordenadas visuais e as coordenadas de referência:

```text
erro_pos = sqrt((x_visual - x_gt)^2 + (y_visual - y_gt)^2)
```

Para a orientação, a diferença angular é normalizada para o intervalo de `-π` a `π`:

```text
erro_yaw = atan2(sin(yaw_visual - yaw_gt), cos(yaw_visual - yaw_gt))
```

A partir dessas diferenças, o relatório calcula RMSE, erro absoluto médio, percentil 95 e erro máximo.

## Arquivos principais

| Arquivo | Função |
|---|---|
| `scripts/top_view_node.py` | Detecta a tag e publica a pose estimada. |
| `src/lar_gazebo/geometry.py` | Reúne os cálculos geométricos e transformações. |
| `scripts/visual_patrol.py` | Movimenta o Husky com base na pose visual. |
| `scripts/evaluate_top_view.py` | Compara a estimativa com o ground truth. |
| `scripts/record_top_view.py` | Grava a imagem anotada da simulação. |
| `scripts/report_top_view.py` | Gera as métricas e os gráficos. |
| `launch/top_view.launch` | Inicia os nós e configura o experimento. |
| `worlds/lar_top_view.world` | Define o ambiente utilizado na simulação. |
| `results/poses.csv` | Guarda as poses comparadas e seus erros. |
| `results/RESULTADOS.md` | Apresenta os resultados detalhados. |

Os dados desta execução também estão em [`metrics.json`](results/metrics.json). O [vídeo](results/demo.mp4) mostra a imagem de topo com a identificação da tag e a pose estimada.

## Testes

Dentro do ambiente ROS, com o workspace configurado:

```bash
cd /ws/src/lar_gazebo
python3 -m unittest discover -s tests -v
```

Os testes verificam cálculos de geometria, recuperação de pose e o comportamento do gravador. Eles complementam, mas não substituem, a execução integrada no Gazebo.

## Limitações

- Os resultados apresentados vêm de **uma execução em simulação**.
- A estimativa assume movimento planar, câmera calibrada e altura conhecida da AprilTag.
- Perdas de detecção e oclusões podem reduzir a disponibilidade das poses.
- A precisão observada no Gazebo não pode ser transferida diretamente para uma câmera real sem novos testes e calibração.
- O controle é baseado em pontos fixos e não realiza desvio de obstáculos.

## Projeto de origem e referências

Este trabalho utiliza como base o ambiente de simulação do [Laboratório de Robótica da UFBA](https://github.com/lar-deeufba/lar_gazebo), disponibilizado sob licença BSD-2-Clause. A documentação do projeto original foi mantida em [`doc/README_original.md`](doc/README_original.md).

Referências de implementação: [detecção de marcadores no OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) e [plugins ROS do Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins).
