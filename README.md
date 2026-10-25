# Localização visual de topo com AprilTag — LaR

Sistema ROS 1 para estimar a pose planar `(x, y, yaw)` do Husky no laboratório
LaR/UFBA a partir de uma câmera fixa de topo e uma AprilTag **36h11, ID 42**
montada horizontalmente sobre o robô.

A visão e o controle **não leem a pose do Gazebo, odometria ou GPS**. O ground
truth é consumido exclusivamente pelo nó de avaliação. Os sensores e o filtro
internos do Husky podem continuar ativos como dependências do simulador, mas
não participam da localização visual nem do controle do percurso.

Execução validada no Gazebo: **2.521 poses**, RMSE de **1,78 cm** em posição
e **0,72°** em yaw. Veja [resultados](results/RESULTADOS.md) e
[vídeo demonstrativo](results/demo.mp4).

## Instalação e execução com Docker

Requisitos: Linux com Docker Engine, Docker Compose e acesso ao display X11
(ou Xvfb). Mesmo com `gui:=false`, a câmera do Gazebo precisa de renderização.
A imagem usa Ubuntu 20.04, ROS Noetic, Gazebo Classic 11 e OpenCV 4.2 com `aruco`.

```bash
git clone https://github.com/Devjoaolima/lar_gazebo_joao_lima.git
cd lar_gazebo_joao_lima
chmod +x scripts/*.sh
./scripts/build.sh
./scripts/run_top_view.sh
```

O comando inicia o laboratório, o robô com a tag, a câmera, o estimador, o
controlador visual, o avaliador e a gravação. A simulação inicia despausada.
Para usar somente o servidor, mantendo acesso a um display de renderização:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/run_top_view.sh gui:=false
```

Se o driver gráfico não for compatível com o contêiner, use
`LIBGL_ALWAYS_SOFTWARE=1` também com a interface aberta. O script entra na raiz
do projeto automaticamente. O terminal permanece ocupado pelo `roslaunch`.

Em outro terminal:

```bash
./scripts/shell.sh
# Dentro do contêiner:
rostopic echo /top_view/done
rosrun image_view image_view image:=/top_view/annotated
```

Quando `/top_view/done` for `true`, o robô para e o gravador fecha o MP4.
A imagem Docker inclui FFmpeg para converter automaticamente o vídeo para H.264;
aguarde o gravador encerrar antes de parar a simulação. Na instalação nativa,
instale `ffmpeg` para obter a mesma conversão (sem ele, a gravação usa MPEG-4).

Encerre o launch com Ctrl+C para fechar a avaliação. Em seguida gere o relatório:

```bash
./scripts/shell.sh
# Dentro do contêiner:
rosrun lar_gazebo report_top_view.py /ws/src/lar_gazebo/results/poses.csv
```

Cada execução sobrescreve `results/poses.csv` e `results/demo.mp4`; copie os
resultados antes de repetir. Alternativamente forneça `output:=/caminho` ao
launch (use um diretório montado no host para conservar os arquivos).

## Instalação nativa

Em um workspace ROS Noetic/Ubuntu 20.04, clone este repositório em `src/`, execute
`rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic` e
`catkin build`. Depois de `source devel/setup.bash`:

```bash
roslaunch lar_gazebo top_view.launch
```

Dependências de visão e gráficos: `python3-opencv`, `python3-numpy`,
`python3-matplotlib`, `ros-noetic-cv-bridge`, `ros-noetic-message-filters`,
`ros-noetic-tf` e `ros-noetic-image-view`, além dos pacotes Husky/Gazebo
já declarados em `package.xml` e no Dockerfile.

## Método e referenciais

O referencial fixo é **`world`**, o próprio referencial do chão definido pelo
arquivo SDF: z aponta para cima, x/y ficam no plano do laboratório, unidades em
metros e ângulos em radianos. Não é criado um referencial ancorado na pose inicial
do robô e não é aplicado ajuste posterior da trajetória ao ground truth.

A câmera fixa está em `(4.65, 3.0, 9.0)` m, olhando verticalmente para baixo,
com imagem de 1600×1200 a 15 Hz e campo horizontal de 1.4 rad. A calibração
intrínseca vem de `/overhead/camera_info`. A extrínseca é obtida da instalação
fixa especificada no SDF e fornecida no launch, sem consultas à pose simulada.

Para coordenadas ópticas (x à direita, y abaixo, z para frente):

```text
T_world_camera = [ 0 -1  0  4.65 ]
                 [-1  0  0  3.00 ]
                 [ 0  0 -1  9.00 ]
                 [ 0  0  0  1.00 ]
```

A placa tem 0.50 m de lado, incluindo margem branca. O contorno preto da tag
mede **0.40 m**. Seu centro fica a `(0,0,0.55)` m em relação a `base_link`, com
os eixos da tag alinhados aos do robô. A altura nominal de `base_link` em relação
ao piso é 0.192 m (geometria do Husky), portanto o plano da tag é **z=0.742 m**.
Esse valor é uma calibração mecânica fixa, não uma medição do ground truth.

O processamento é:

1. Correção gamma fixa para compensar a baixa intensidade da textura sob a
   iluminação do mundo e detecção OpenCV `DICT_APRILTAG_36h11` com refinamento
   subpixel dos cantos; apenas o ID 42 é aceito. A separação mínima entre
   candidatos é reduzida para 0.01 para que o contorno próximo do robô não
   elimine o candidato correspondente à tag.
2. `solvePnP` com os quatro cantos e o lado conhecido verifica solução à frente
   da câmera e erro de reprojeção de até 2 px.
3. Os raios dos cantos, corrigidos da distorção, são transformados para `world`
   e intersectados com o plano calibrado da tag. O centro fornece a posição;
   as arestas ordenadas da tag fornecem a orientação yaw, sem ambiguidade de 90°.
4. Compensa-se a montagem da tag para publicar a pose de `base_link`, segundo
   `T_world_base = T_world_tag × inverse(T_base_tag)`.

A hipótese planar evita a instabilidade de profundidade/inclinação do PnP para
uma tag pequena vista de longe. A altura z é fixa; roll/pitch são assumidos zero.
O objetivo avaliado é pose **SE(2)**, não reconstrução de movimento 6D. Em um
laboratório real, intrínsecos, extrínsecos e altura precisam ser calibrados por
medidas físicas ou marcadores de referência fixos no chão.

## Percurso e segurança operacional

O controlador segue os pontos `(5.65,3.0)`, `(5.65,3.8)`, `(4.65,3.8)` e
`(4.65,3.0)` dentro da área livre do laboratório. Usa somente `/top_view/pose`,
com velocidade linear máxima de 0.2 m/s, angular de 0.5 rad/s e tolerância de
0.12 m por ponto. Para sem pose visual recente (0.5 s), ao concluir ou ao encerrar.

Os pontos são uma rota previamente escolhida para este cenário, sem planejamento
global nem desvio de obstáculos. Não se deve transferir o controlador diretamente
para um robô físico. Para mudar a rota, configure o parâmetro privado `waypoints`
do nó `visual_patrol` no launch. Para observar sem movimento, use `drive:=false`;
para desativar a gravação, `record:=false`.

## Avaliação e resultados

O plugin `libgazebo_ros_p3d.so` publica a pose verdadeira de `base_link` em
`/evaluation/ground_truth`, com `frameName=world` e offsets nulos. O antigo
plugin em `gazebo_ground_truth.urdf`, que desloca a origem, não é usado neste
experimento.

O avaliador usa o **timestamp da imagem** e interpola duas amostras de ground
truth que envolvem esse instante (intervalo máximo de 0.1 s). Não compara a
imagem atrasada com a última pose disponível. A interpolação de yaw e o erro
angular usam o menor arco em `[-pi,pi]`.

```text
erro_posição = sqrt((x_visual-x_gt)^2 + (y_visual-y_gt)^2)
erro_yaw = atan2(sin(yaw_visual-yaw_gt), cos(yaw_visual-yaw_gt))
```

Arquivos produzidos:

- [`results/poses.csv`](results/poses.csv): poses pareadas e erros por instante;
- [`results/metrics.json`](results/metrics.json): RMSE, MAE, percentil 95 e máximo;
- [`results/errors.png`](results/errors.png): trajetórias e erros ao longo do tempo;
- [`results/demo.mp4`](results/demo.mp4): vídeo real da câmera, com detecções,
  posição estimada e tempo simulado. A reprodução usa 15 quadros/s; perdas de
  quadros podem comprimir trechos, e o tempo sobreposto identifica os instantes.
- [`results/RESULTADOS.md`](results/RESULTADOS.md): resumo da execução incluída.

**Vídeo demonstrativo:** [abrir ou baixar o MP4](results/demo.mp4).

As métricas são condicionadas às detecções aceitas e aos pares temporais válidos;
ausências de tag não geram poses inventadas nem erros artificiais. Iluminação,
oclusão, tamanho em pixels e erro na extrínseca/altura afetam a precisão. Os
resultados da simulação ideal não garantem o mesmo erro em uma câmera real.

## Código e testes

- `scripts/top_view_node.py`: imagem/calibração → pose visual e imagem anotada;
- `src/lar_gazebo/geometry.py`: PnP, projeção de raios e transformações;
- `scripts/visual_patrol.py`: controle usando exclusivamente visão;
- `scripts/evaluate_top_view.py`: único consumidor do ground truth;
- `scripts/record_top_view.py`: gravação do vídeo demonstrativo;
- `scripts/report_top_view.py`: métricas e gráficos a partir do CSV;
- `launch/top_view.launch`, `worlds/lar_top_view.world`: integração do cenário;
- `husky_urdf_extras/top_view.urdf`: tag solidária ao robô e plugin de avaliação.

Dentro do ambiente ROS, com o workspace carregado:

```bash
cd /ws/src/lar_gazebo
python3 -m unittest discover -s tests -v
```

Os testes projetam cantos a partir de poses conhecidas e verificam a recuperação
de posição, yaw, deslocamento da montagem e a passagem angular de ±180°. Também verificam que imagens na fila não reabrem o vídeo após seu encerramento.
Não substituem a execução integrada no Gazebo.

## Créditos

Ambiente original: [lar-deeufba/lar_gazebo](https://github.com/lar-deeufba/lar_gazebo).
Mantidos os modelos e a licença BSD do projeto. Instruções originais disponíveis
em [doc/README_original.md](doc/README_original.md).

Referências técnicas: [detecção de marcadores OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
e [plugins ROS do Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins).
