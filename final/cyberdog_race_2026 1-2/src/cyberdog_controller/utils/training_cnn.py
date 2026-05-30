import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

class ArrowDataset(Dataset):
    def __init__(self, data, targets):
        self.data = data
        self.targets = targets

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx], self.targets[idx]
    

class ArrowDataLoader(DataLoader):
    def __init__(self, dataset, batch_size=32, shuffle=True, num_workers=0):
        super().__init__(dataset, batch_size=batch_size, shuffle=shuffle, num_workers=num_workers)

class ResidualBlock(nn.Module):
    def __init__(self, channels):
        super(ResidualBlock, self).__init__()
        self.conv1 = nn.Conv2d(channels, channels, kernel_size=3, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(channels)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(channels, channels, kernel_size=3, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(channels)

    def forward(self, x):
        identity = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out += identity
        out = self.relu(out)
        return out


class SimpleCNN(nn.Module):
    def __init__(self, in_channels=3, num_classes=2):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        self.resblock1 = ResidualBlock(64)
        self.resblock2 = ResidualBlock(64)
        self.adapt_pool = nn.AdaptiveAvgPool2d((1, 1))
        self.fc = nn.Linear(64, num_classes)

    def forward(self, x):
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)
        out = self.pool(out)
        out = self.resblock1(out)
        out = self.resblock2(out)
        out = self.adapt_pool(out)
        out = out.view(out.size(0), -1)
        out = self.fc(out)
        return out


def criterion(logits, target):
    return nn.CrossEntropyLoss()(logits, target)

if __name__ == '__main__':
    # Dataset parameters
    in_channels = 3
    num_classes = 2
    num_samples = 1000
    image_size = (224, 224)

    # Hyperparameters
    seed = 42
    learning_rate = 0.001
    batch_size = 32
    epochs = 60

    torch.manual_seed(seed)
    model = SimpleCNN()
    dataset = ArrowDataset()
    dataloader = ArrowDataLoader(dataset, batch_size=batch_size)
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    for i in tqdm(range(epochs), desc="Training Epochs"):
        for data, target in dataloader:
            optimizer.zero_grad()
            logits = model(data)
            loss = criterion(logits, target)
            loss.backward()
            optimizer.step()

            tqdm.write(f'Epoch {i+1}/{epochs}, Loss: {loss.item()}')

